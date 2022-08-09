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
  using Base = Bridge;                                                                 // Access specifier
  RaisimBridge(Model model, double dt, double g, double mu, bool fixed, bool record);  // constructor
  void Init() override;
  void SimRun(Eigen::Matrix<double, 5, 1> u) override;
  void End() override;
  Eigen::Matrix<double, 14, 1> jointNominalConfig;

 private:
  raisim::World world;
  raisim::RaisimServer server;
  // raisim::ArticulatedSystem* bot;// MJCF version
  std::vector<raisim::ArticulatedSystem*> bot;  // URDF version
};