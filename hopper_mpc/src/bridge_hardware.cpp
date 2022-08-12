#include "hopper_mpc/bridge_hardware.h"
#include <cstring>
#include <filesystem>
#include <iostream>

HardwareBridge::HardwareBridge(Model model, double dt, double g, double mu, bool fixed, bool record)
    : Base(model, dt, g, mu, fixed, record) {
  ODriveCan ODriveCAN1(Channel::CAN1, BandRate::BAUD_1M);  // Channel 1
  node_id_q0 = 0;
  node_id_rw1 = 2;

  ODriveCan ODriveCAN2(Channel::CAN2, BandRate::BAUD_1M);  // Channel 2
  node_id_q2 = 1;
  node_id_rw1 = 3;

  ODriveCan ODriveCAN3(Channel::CAN3, BandRate::BAUD_1M);  // Channel 3
  node_id_rwz = 4;
}

void HardwareBridge::Init() {
  ODriveCAN1.initialize();
  ODriveCAN2.initialize();
  ODriveCAN3.initialize();
  Home(ODriveCAN1, node_id_q0, 1);
  Home(ODriveCAN2, node_id_q2, -1);
  if (calibrate == true) {
    a_cal_ = GetPosition();  // read the encoder positions at home
  }
  // after calibration, prepare for pos control
  SetPosCtrl(ODriveCAN1, node_id_q0, model.q_init(0));
  SetPosCtrl(ODriveCAN2, node_id_q2, model.q_init(2));
}

void HardwareBridge::SetPosCtrl(ODriveCan ODrive, int node_id, double q_init) {
  // Initialize Odrives into position control
  ODrive.SetControllerModes(node_id, ODriveCan::POSITION_CONTROL);
  ODrive.SetPos(node_id, q_init);
}

void HardwareBridge::Subscribe(ODriveCan ODrive, int node_id) {
  ODrive.subscribeTopic(0x22, [&](const TPCANMsg& msg) {
    msgBuffer.writeMsgToBuffer(msg);
    {
      std::lock_guard<std::mutex> lock(receivedMutex);
      msgReceived = true;
    }
    receivedCondition.notify_all();
  });
  // can2.initialize();
  // std::cout << "Start can..."
  //           << "\n";
  std::thread t(worker);
  t.join();
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

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointPosition() {
  q = TurnsToRadians(Odrive.position) + model.q_init_cal - a_cal_;  // change encoder reading to match model
  return q;
}

retVals HardwareBridge::SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) {
  q = GetJointPosition();
  dq = GetJointVelocity();
  // Torque vs Position Control for Legs
  self.X [0:3] = p_base;
  self.X [3:7] = Q_base;
  self.X [7:10] = Z(Q_inv(Q_base), velocities[0]);
  // linear vel world->body frame
  self.X [10:] = Z(Q_inv(Q_base), velocities[1]);  // angular vel world->body frame

  return retVals{X_, qa_, dqa_};
}

void HardwareBridge::End() {
  SetPosCtrl(ODrive_q0, node_id_q0, model.q_init(0));
  SetPosCtrl(ODrive_q2, node_id_q2, model.q_init(2));
}

// Utility functions
double HardwareBridge::TurnsToRadians(double turns) {
  return 2 * M_PI * turns;
}