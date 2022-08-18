#include "hopper_mpc/bridge_raisim.h"
#include <filesystem>
#include <iostream>

RaisimBridge::RaisimBridge(Model model_, double dt_, bool fixed_, bool record_) : Base(model_, dt_, fixed_, record_), server(&world) {}

void RaisimBridge::Init() {
  // char tmp[256];
  // getcwd(tmp, 256);
  // std::cout << "Current working directory: " << tmp << std::endl;
  raisim::World::setActivationKey("/home/vscode/activation.raisim");

  // TODO: Add rotor inertia
  // MJCF version
  // std::string s = "/workspaces/RosDockerWorkspace/devel/lib/hopper_mpc/hopper_mpc";
  // char* dir = new char[200];  // just needs to be larger than the actual string
  // strcpy(dir, s.c_str());
  // auto binaryPath = raisim::Path::setFromArgv(dir);
  // raisim::World world(binaryPath.getDirectory() + "/../../../src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08_mjcf.xml");
  // bot = static_cast<raisim::ArticulatedSystem*>(world.getObject("base_link"));

  // URDF version
  robot.push_back(world.addArticulatedSystem("/workspaces/RosDockerWorkspace/src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08.urdf"));
  bot = robot.back();
  auto ground = world.addGround();
  world.setTimeStep(dt);
  /// launch raisim server for visualization. Can be visualized in raisimUnity
  server.launchServer();
  server.focusOn(bot);

  // robot state
  std::cout << bot->getGeneralizedCoordinateDim() << '\n';
  // jointNominalConfig(bot->getGeneralizedCoordinateDim());
  // jointNominalConfig.setZero();
  // jointNominalConfig[1] = 0.1;
  //

  // bot->setGeneralizedCoordinate({0, 0, 0, 0, 0, 0, 0});  // first 7 elements are pos and quaternion
  // bot->setGeneralizedCoordinate({0, 0, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});  // first 7 elements are pos and quaternion
  bot->setGeneralizedCoordinate({0, 0, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});  // first 7 elements are pos and quaternion
  bot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
  // bot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  n_dof = bot->getDOF();
  // std::cout << n_dof << std::endl;
  jointState.resize(n_dof);
  jointForce.resize(n_dof);
  jointPgain.resize(n_dof);
  jointDgain.resize(n_dof);
  jointPgain.setZero();
  jointDgain.setZero();
  bot->setPdGains(jointPgain, jointDgain);

  // std::vector<std::__cxx11::basic_string<char>> path = bot->getMovableJointNames();
  // for (auto i : path) std::cout << i << ' ' << std::endl;
  bot->ignoreCollisionBetween(0, 1);
  bot->ignoreCollisionBetween(0, 2);
  bot->ignoreCollisionBetween(0, 3);
  bot->ignoreCollisionBetween(0, 4);
  bot->ignoreCollisionBetween(1, 2);
  bot->ignoreCollisionBetween(1, 3);
  bot->ignoreCollisionBetween(1, 4);
  bot->ignoreCollisionBetween(3, 4);
  bot->ignoreCollisionBetween(2, 3);
  bot->ignoreCollisionBetween(2, 4);

  // initialize variables
  qa_cal << model.q_init(0), model.q_init(2);
}

retVals RaisimBridge::SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) {
  raisim::MSLEEP(2);
  jointForce << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 1;
  // jointForce << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 1;
  // bot->setGeneralizedForce(jointForce);
  // bot->setGeneralizedCoordinate(jointNominalConfig);
  // bot->setGeneralizedForce(jointNominalConfig);
  // std::this_thread::sleep_for(std::chrono::microseconds(1000));
  server.integrateWorldThreadSafe();
  // world.integrate();
  qa = qa_raw + qa_cal;  // Correct the angle. Make sure this only happens once per time step
  return retVals{X, qa, dqa};
}

void RaisimBridge::End() {
  server.killServer();
}