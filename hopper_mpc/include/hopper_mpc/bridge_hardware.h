#pragma once

#include "hopper_mpc/model.h"
// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"

class HardwareBridge {                                                                // The class
 public:                                                                              // Access specifier
  HardwareBridge(Model model, float dt, float g, float mu, bool fixed, bool record);  // constructor
  Model model;
  float dt;
  float g;
  float mu;
  bool fixed;
  bool record;
  void Init(Eigen::Vector4d init_q);
  void Calibrate();
  void SimRun(Eigen::Matrix<double, 5, 1> u);
  void End();

 private:
  ODriveCan ODrive_q0;  // leg links
  ODriveCan ODrive_q2;
};