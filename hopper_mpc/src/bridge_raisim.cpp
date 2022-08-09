#include "hopper_mpc/bridge_raisim.h"
#include <filesystem>
#include <iostream>

RaisimBridge::RaisimBridge(Model model, double dt, double g, double mu, bool fixed, bool record)
    : Base(model, dt, g, mu, fixed, record), server(&world) {}

void RaisimBridge::Init() {
  // char tmp[256];
  // getcwd(tmp, 256);
  // std::cout << "Current working directory: " << tmp << std::endl;
  raisim::World::setActivationKey("/home/vscode/activation.raisim");

  // MJCF version
  // std::string s = "/workspaces/RosDockerWorkspace/devel/lib/hopper_mpc/hopper_mpc";
  // char* dir = new char[150];  // just needs to be larger than the actual string
  // strcpy(dir, s.c_str());
  // auto binaryPath = raisim::Path::setFromArgv(dir);
  // raisim::World world(binaryPath.getDirectory() + "/../../../src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08.xml");
  // bot = static_cast<raisim::ArticulatedSystem*>(world.getObject("base_link"));

  // URDF version
  bot.push_back(
      world.addArticulatedSystem("/workspaces/RosDockerWorkspace/src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08_obj.urdf"));

  auto ground = world.addGround();  // what's wrong with the units???
  // TODO: Try modifying meshes to reduce complexity--might be causing the contact issues
  world.setTimeStep(dt_);
  // raisim::Vec<3> gravity = world.getGravity();
  // std::cout << gravity << '\n';
  /// launch raisim server for visualization. Can be visualized in raisimUnity
  server.launchServer();
  // server.focusOn(bot); // MJCF version
  server.focusOn(bot.back());  // URDF version
  // std::cout << typeid(server).name() << '\n';

  // robot state
  std::cout << bot.back()->getGeneralizedCoordinateDim() << '\n';
  // jointNominalConfig(bot.back()->getGeneralizedCoordinateDim());
  // jointNominalConfig.setZero();
  // jointNominalConfig[1] = 0.1;
  //
  // bot->setGeneralizedCoordinate({0, 0, 0, 0, 0, 0, 0});  // first 7 elements are pos and quaternion
  bot.back()->setGeneralizedCoordinate({0, 0, 0.7, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});  // first 7 elements are pos and quaternion
}

void RaisimBridge::SimRun(Eigen::Matrix<double, 5, 1> u) {
  raisim::MSLEEP(1);
  // bot.back()->setGeneralizedCoordinate(jointNominalConfig);
  // bot->setGeneralizedForce(jointNominalConfig);
  // std::this_thread::sleep_for(std::chrono::microseconds(1000));
  server.integrateWorldThreadSafe();
  // world.integrate();
}

void RaisimBridge::End() {
  server.killServer();
}