#include "hopper_mpc/bridge_raisim.h"
#include <filesystem>
#include <iostream>

RaisimBridge::RaisimBridge(Model model, float dt, float g, float mu, bool fixed, bool record) : server(&world) {
  // constructor
  model = model;
  dt = dt;
  g = g;
  mu = mu;
  fixed = fixed;
  record = record;
}

void RaisimBridge::Init(Eigen::Vector4d init_q) {
  raisim::World::setActivationKey("/home/vscode/activation.raisim");
  // char tmp[256];
  // getcwd(tmp, 256);
  // std::cout << "Current working directory: " << tmp << std::endl;
  auto anymal =
      world.addArticulatedSystem("/workspaces/RosDockerWorkspace/src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08_obj.urdf");
  auto ground = world.addGround();
  world.setTimeStep(dt);

  /// launch raisim server for visualization. Can be visualized on raisimUnity

  server.launchServer();
  std::cout << typeid(server).name() << '\n';
}

void RaisimBridge::SimRun(Eigen::Matrix<double, 5, 1> u) {
  raisim::MSLEEP(2);
  // server.integrateWorldThreadSafe();
}

void RaisimBridge::End() {
  // server.killServer();
}