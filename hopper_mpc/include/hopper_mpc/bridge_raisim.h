#pragma once

#include "hopper_mpc/model.h"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"

class RaisimBridge {                                                                // The class
 public:                                                                            // Access specifier
  RaisimBridge(Model model, float dt, float g, float mu, bool fixed, bool record);  // constructor
  Model model;
  float dt;
  float g;
  float mu;
  bool fixed;
  bool record;
  void Init(Eigen::Vector4d init_q);
  void SimRun(Eigen::Matrix<double, 5, 1> u);
  void End();

 private:
  raisim::World world;
  raisim::RaisimServer server;
};