#include "hopper_mpc/bridge_hardware.h"
#include <cstring>
#include <filesystem>
#include <iostream>

HardwareBridge::HardwareBridge(Model model, double dt, double g, double mu, bool fixed, bool record)
    : Base(model, dt, g, mu, fixed, record) {
  ODriveCan ODriveCAN4(Channel::CAN1, BandRate::BAUD_1M);  // Channel 1
  node_id_q0 = 0;
  node_id_rwl = 2;

  ODriveCan ODriveCAN3(Channel::CAN2, BandRate::BAUD_1M);  // Channel 2
  node_id_q2 = 1;
  node_id_rwr = 3;

  ODriveCan ODriveCAN2(Channel::CAN3, BandRate::BAUD_1M);  // Channel 3
  node_id_rwz = 4;

  // init variables
  X_.setZero();
  qa_.setZero();
  dqa_.setZero();
}

void HardwareBridge::Init() {
  ODriveCAN1.initialize();
  ODriveCAN2.initialize();
  ODriveCAN3.initialize();
  Home(ODriveCAN1, node_id_q0, 1);
  Home(ODriveCAN2, node_id_q2, -1);
  if (calibrate == true) {
    q_offset_ = GetJointPosRaw();  // read the encoder positions at home
  }
  // after calibration, prepare for pos control
  SetPosCtrl(ODriveCAN1, node_id_q0, model_.q_init(0));
  SetPosCtrl(ODriveCAN2, node_id_q2, model_.q_init(2));
  ctrlMode_prev = "Pos";
}

void HardwareBridge::SetPosCtrl(ODriveCan ODrive, int node_id, double q_init) {
  // set Odrives to position control
  ODrive.SetPos(node_id, q_init);
  ODrive.SetControllerModes(node_id, ODriveCan::POSITION_CONTROL);
}

void HardwareBridge::SetTorCtrl(ODriveCan ODrive, int node_id) {
  // set Odrives to torque control
  ODrive.SetTorque(node_id, 0);
  ODrive.SetControllerModes(node_id, ODriveCan::TORQUE_CONTROL);
}

void HardwareBridge::Home(ODriveCan ODrive, int node_id, int dir) {
  // Tell ODrive to find the leg position
  const double vel = 0.5;
  ODrive.SetControllerModes(node_id, ODriveCan::VELOCITY_CONTROL);
  ODrive.RunState(node_id, ODriveCan::AXIS_STATE_CLOSED_LOOP_CONTROL);
  ODrive.SetVelocity(node_id, vel * dir);
  // assume that at the end of 5 seconds it has found home
  std::this_thread::sleep_for(std::chrono::seconds(5));
  // TODO: More complex but reliable homing procedure?
  ODrive.SetVelocity(node_id, 0);  // stop the motor
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointPos() {
  Eigen::Matrix<double, 5, 1> qa;
  qa(0) = ODriveCAN4.GetPosition(node_id_q0) * 2 * M_PI + model_.qa_home(0);
  qa(1) = ODriveCAN3.GetPosition(node_id_q2) * 2 * M_PI + model_.qa_home(1);
  qa(2) = ODriveCAN3.GetPosition(node_id_rwr) * 2 * M_PI;
  qa(3) = ODriveCAN4.GetPosition(node_id_rwl) * 2 * M_PI;
  qa(4) = ODriveCAN2.GetPosition(node_id_rwz) * 2 * M_PI;
  qa = qa - q_offset_;  // change encoder reading to match model
  return qa;
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointPosRaw() {
  Eigen::Matrix<double, 5, 1> qa;
  qa(0) = ODriveCAN4.GetPosition(node_id_q0) * 2 * M_PI;
  qa(1) = ODriveCAN3.GetPosition(node_id_q2) * 2 * M_PI;
  qa(2) = ODriveCAN3.GetPosition(node_id_rwr) * 2 * M_PI;
  qa(3) = ODriveCAN4.GetPosition(node_id_rwl) * 2 * M_PI;
  qa(4) = ODriveCAN2.GetPosition(node_id_rwz) * 2 * M_PI;
  return qa;
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointVel() {
  Eigen::Matrix<double, 5, 1> dqa;
  dqa(0) = ODriveCAN4.GetVelocity(node_id_q0) * 2 * M_PI;  // TODO: is odrive velocity in RPS?
  dqa(1) = ODriveCAN3.GetVelocity(node_id_q2) * 2 * M_PI;
  dqa(2) = ODriveCAN3.GetVelocity(node_id_rwr) * 2 * M_PI;
  dqa(3) = ODriveCAN4.GetVelocity(node_id_rwl) * 2 * M_PI;
  dqa(4) = ODriveCAN2.GetVelocity(node_id_rwz) * 2 * M_PI;
  return dqa;
}

retVals HardwareBridge::SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) {
  // Torque vs Position Control for Legs
  if (ctrlMode == "Pos" && ctrlMode_prev != "Pos") {
    SetPosCtrl(ODriveCAN4, node_id_q0, qla_ref(0));
    SetPosCtrl(ODriveCAN3, node_id_q2, qla_ref(1));
  } else if (ctrlMode == "Torque" && ctrlMode_prev != "Torque") {
    SetTorCtrl(ODriveCAN4, node_id_q2);
    SetTorCtrl(ODriveCAN3, node_id_q2);
  }
  qa_ = GetJointPos();
  dqa_ = GetJointVel();
  // X.block<3, 1>(0, 0) = p_base;
  // X.block<4, 1>(3, 0) = Q_base;
  // X.block<3, 1>(7, 0) = Z(Q_inv(Q_base), velocities[0]);   // linear vel world->body frame
  // X.block<3, 1>(10, 0) = Z(Q_inv(Q_base), velocities[1]);  // angular vel world->body frame
  ctrlMode_prev = ctrlMode;
  return retVals{X_, qa_, dqa_};
}

void HardwareBridge::End() {
  SetPosCtrl(ODriveCAN1, node_id_q0, model_.q_init(0));
  SetPosCtrl(ODriveCAN2, node_id_q2, model_.q_init(2));
}

// Utility functions
double HardwareBridge::TurnsToRadians(double turns) {
  return 2 * M_PI * turns;
}

// double HardwareBridge::Mapper(double posEst, double velEst) {
//   // This would have to be a class?
//   // Or part of the ODrive class
//   // Maps encoder abs position (turns) to linear space
//   double dx = posEst - posEst_prev;
//   posEst_prev = posEst;
//   // This might not work depending on thread timing...
//   if (sign(dx) < sign(velEst)) {
//     count += 1;
//   } else if (sign(dx) > sign(velEst)) {
//     count -= 1;
//   }
//   double q = posEst + count;
//   return q;
// }