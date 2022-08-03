#include "hopper_mpc/bridge_hardware.h"
#include <cstring>
#include <filesystem>
#include <iostream>
#include "hopper_can_interface/ODriveCan.h"

HardwareBridge::HardwareBridge(Model model, float dt, float g, float mu, bool fixed, bool record) {
  // constructor
  model = model;
  dt = dt;
  g = g;
  mu = mu;
  fixed = fixed;
  record = record;
  ODriveCan ODrive_q0(Channel::CAN1, BandRate::BAUD_1M);
  ODriveCan ODrive_rw1(Channel::CAN1, BandRate::BAUD_1M);

  ODriveCan ODrive_q2(Channel::CAN2, BandRate::BAUD_1M);
  ODriveCan ODrive_rw2(Channel::CAN2, BandRate::BAUD_1M);

  ODriveCan ODrive_rwz(Channel::CAN3, BandRate::BAUD_1M);

  // CAN bus node ids
  node_id_q0 = 0;
  node_id_q2 = 1;
  node_id_rw1 = 2;
  node_id_rw2 = 3;
  node_id_rwz = 4;
}

void HardwareBridge::Init(Eigen::Vector4d init_q) {
  ODrive_q0.initialize();
  ODrive_q2.initialize();
  Calibrate(ODrive_q0, node_id_q0, 1);
  Calibrate(ODrive_q2, node_id_q2, -1);
  // after calibration, prepare for torque control
  SetPosCtrl(ODrive_q0, node_id_q0, -30);
  SetPosCtrl(ODrive_q2, node_id_q2, -150);
}

void HardwareBridge::SetPosCtrl(ODriveCan ODrive, int node_id, double q_init) {
  // Initialize Odrives into position control
  ODrive.SetControllerModes(node_id, ODriveCan::POSITION_CONTROL);
  ODrive.SetTorque(node_id, q_init);
}

void HardwareBridge::Calibrate(ODriveCan ODrive, int node_id, int dir) {
  // Tell ODrive to find the leg position
  const double vel = 0.5;
  ODrive.SetControllerModes(node_id, ODriveCan::VELOCITY_CONTROL);
  ODrive.RunState(node_id, ODriveCan::AXIS_STATE_CLOSED_LOOP_CONTROL);
  ODrive.SetVelocity(node_id, vel * dir);
  // assume that at the end of 5 seconds it has found home
  std::this_thread::sleep_for(std::chrono::seconds(5));
  // TODO: More complex but reliable homing procedure?
}

void HardwareBridge::SimRun(Eigen::Matrix<double, 5, 1> u) {
  raisim::MSLEEP(2);
  // server.integrateWorldThreadSafe();
}

void HardwareBridge::End() {
  // server.killServer();
}