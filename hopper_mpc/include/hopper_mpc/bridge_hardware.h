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
  void Init();
  void Calibrate();
  void SimRun(Eigen::Matrix<double, 5, 1> u);
  void End();

 private:
  ODriveCan ODrive_q0;  // leg links
  ODriveCan ODrive_q2;
  ODriveCan ODrive_rw1;
  ODriveCan ODrive_rw2;
  ODriveCan ODrive_rwz;
  double node_id_q0;
  double node_id_q2;
  double node_id_rw1;
  double node_id_rw2;
  double node_id_rwz;
  Eigen::Matrix<double, 5, 1> a_cal_;
};