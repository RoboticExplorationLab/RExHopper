#pragma once

#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"

class RaisimBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                                // Access specifier
  RaisimBridge(Model model_, double dt_, bool fixed_, bool record_);  // constructor
  void Init() override;
  retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) override;
  void End() override;
  Eigen::Matrix<double, 14, 1> jointNominalConfig;
  int n_dof;
  Eigen::VectorXd jointState, jointForce, jointPgain, jointDgain;

 private:
  raisim::World world;
  raisim::RaisimServer server;

  std::vector<raisim::ArticulatedSystem*> robot;  // needed for URDF version
  raisim::ArticulatedSystem* bot;
};