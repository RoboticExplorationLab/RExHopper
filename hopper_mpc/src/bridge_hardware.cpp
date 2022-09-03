#include "hopper_mpc/bridge_hardware.h"
#include <cstring>
#include <filesystem>
#include <iostream>

HardwareBridge::HardwareBridge(Model model_, double dt_, bool fixed_, bool record_) : Base(model_, dt_, fixed_, record_) {}

void HardwareBridge::Init() {
  ODriveCANleft.reset(new ODriveCan(Channel::CAN4, BandRate::BAUD_1M));
  node_id_q0 = 0;
  node_id_rwl = 3;

  ODriveCANright.reset(new ODriveCan(Channel::CAN3, BandRate::BAUD_1M));
  node_id_q2 = 1;
  node_id_rwr = 2;

  ODriveCANyaw.reset(new ODriveCan(Channel::CAN1, BandRate::BAUD_1M));
  node_id_rwz = 4;

  // init variables
  qa.setZero();
  dqa.setZero();

  std::map<std::string, int> mapCANleft;
  mapCANleft.insert(std::make_pair("q0", node_id_q0));
  mapCANleft.insert(std::make_pair("rwl", node_id_rwl));
  std::map<std::string, int> mapCANright;
  mapCANright.insert(std::make_pair("q2", node_id_q2));
  mapCANright.insert(std::make_pair("rwr", node_id_rwr));
  std::map<std::string, int> mapCANyaw;
  mapCANyaw.insert(std::make_pair("rwz", node_id_rwz));

  ODriveCANleft->initialize(mapCANleft);
  ODriveCANright->initialize(mapCANright);
  ODriveCANyaw->initialize(mapCANyaw);

  Home(ODriveCANleft, node_id_q0, -1);
  Home(ODriveCANright, node_id_q2, -1);
  q_offset_ = GetJointPosRaw();  // read the encoder positions at home

  // after calibration, prepare for pos control
  SetPosCtrl(ODriveCANleft, node_id_q0, model.q_init(0));
  SetPosCtrl(ODriveCANright, node_id_q2, model.q_init(2));
  ctrlMode_prev = "Pos";

  p.setZero();
  Q.coeffs() << 0, 0, 0, 1;
  v.setZero();
  w.setZero();
}

void HardwareBridge::SetPosCtrl(std::unique_ptr<ODriveCan>& ODrive, int node_id, double q_init) {
  // set Odrives to position control
  ODrive->SetPosition(node_id, q_init);
  ODrive->SetControllerModes(node_id, ODriveCan::POSITION_CONTROL);
}

void HardwareBridge::SetTorCtrl(std::unique_ptr<ODriveCan>& ODrive, int node_id) {
  // set Odrives to torque control
  ODrive->SetTorque(node_id, 0);
  ODrive->SetControllerModes(node_id, ODriveCan::TORQUE_CONTROL);
}

void HardwareBridge::Home(std::unique_ptr<ODriveCan>& ODrive, int node_id, int dir) {
  // Tell ODrive to find the leg position
  const double vel = 0.5;
  ODrive->SetControllerModes(node_id, ODriveCan::VELOCITY_CONTROL);
  ODrive->RunState(node_id, ODriveCan::AXIS_STATE_CLOSED_LOOP_CONTROL);
  ODrive->SetVelocity(node_id, vel * dir);
  // assume that at the end of 2 seconds it has found home
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // TODO: More complex but reliable homing procedure?
  ODrive->SetVelocity(node_id, 0);  // stop the motor
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointPosRaw() {
  Eigen::Matrix<double, 5, 1> qa;
  qa(0) = ODriveCANleft->GetPosition(node_id_q0) * 2 * M_PI + model.qa_home(0);
  qa(1) = ODriveCANright->GetPosition(node_id_q2) * 2 * M_PI + model.qa_home(1);
  qa(2) = ODriveCANright->GetPosition(node_id_rwr) * 2 * M_PI;
  qa(3) = ODriveCANleft->GetPosition(node_id_rwl) * 2 * M_PI;
  qa(4) = ODriveCANyaw->GetPosition(node_id_rwz) * 2 * M_PI;
  return qa;
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointPos() {
  Eigen::Matrix<double, 5, 1> qa = GetJointPosRaw();
  qa = qa - q_offset_;  // change encoder reading to match model
  return qa;
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointVel() {
  Eigen::Matrix<double, 5, 1> dqa;
  dqa(0) = ODriveCANleft->GetVelocity(node_id_q0) * 2 * M_PI;  // TODO: is odrive velocity in RPS?
  dqa(1) = ODriveCANright->GetVelocity(node_id_q2) * 2 * M_PI;
  dqa(2) = ODriveCANright->GetVelocity(node_id_rwr) * 2 * M_PI;
  dqa(3) = ODriveCANleft->GetVelocity(node_id_rwl) * 2 * M_PI;
  dqa(4) = ODriveCANyaw->GetVelocity(node_id_rwz) * 2 * M_PI;
  return dqa;
}

retVals HardwareBridge::SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) {
  // Torque vs Position Control for Legs
  if (ctrlMode == "Pos" && ctrlMode_prev != "Pos") {
    SetPosCtrl(ODriveCANleft, node_id_q0, qla_ref(0));
    SetPosCtrl(ODriveCANright, node_id_q2, qla_ref(1));
  } else if (ctrlMode == "Torque" && ctrlMode_prev != "Torque") {
    SetTorCtrl(ODriveCANleft, node_id_q0);
    SetTorCtrl(ODriveCANright, node_id_q2);
  }
  qa = GetJointPos();
  dqa = GetJointVel();

  ctrlMode_prev = ctrlMode;
  return retVals{p, Q, v, w, qa, dqa, sh};
}

void HardwareBridge::End() {
  SetPosCtrl(ODriveCANleft, node_id_q0, model.q_init(0));
  SetPosCtrl(ODriveCANright, node_id_q2, model.q_init(2));
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