#include "hopper_mpc/bridge_gazebo.h"
#include "gazebo/glfw3.h"
#include "gazebo/gazebo.h"
// for sleep timers
#include <chrono>
#include <iostream>
#include <thread>


GazeboBridge::GazeboBridge(Model model, float dt, float g, float mu, bool fixed, bool record) {
  // constructor
  model = model;
  dt = dt;
  g = g;
  mu = mu;
  fixed = fixed;
  record = record;
}

void GazeboBridge::Init(Eigen::Vector4d init_q) {

  
}


void GazeboBridge::SimRun(Eigen::Matrix<double, 5, 1> u) {

    d->ctrl[0] = u[0];
    d->ctrl[1] = 0;
    d->ctrl[2] = u[1];
    d->ctrl[3] = 0;
    d->ctrl[4] = u[2];
    d->ctrl[5] = u[3];
    d->ctrl[6] = u[4];

    std::cout << "actuator force: " << (d->actuator_force[0]) << ", " << (d->actuator_force[1]) << ", " << (d->actuator_force[2]) << ", "
              << (d->actuator_force[3]) << ", " << (d->actuator_force[4]) << ", " << (d->actuator_force[5]) << ", "
              << (d->actuator_force[6]) << std::endl;
    // std::cout << "qpos: " << (d->qpos[0]) << ", " << (d->qpos[1]) << ", " << (d->qpos[2]) << ", " << (d->qpos[3]) << ", " << (d->qpos[4])
    //           << ", " << (d->qpos[5]) << ", " << (d->qpos[6]) << std::endl;
    std::cout << "ctrl: " << (d->ctrl[0]) << ", " << (d->ctrl[1]) << ", " << (d->ctrl[2]) << ", " << (d->ctrl[3]) << ", " << (d->ctrl[4])
              << ", " << (d->ctrl[5]) << ", " << (d->ctrl[6]) << std::endl;
}

void GazeboBridge::End() {
  
}