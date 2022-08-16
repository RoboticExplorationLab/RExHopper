#pragma once
#include "Eigen/Dense"
#include "hopper_mpc/model.h"

class Mpc {                                                                      // The class
 public:                                                                         // Access specifier
  Mpc(Model model, double dt, int N, Eigen::Matrix3d Jinv, Eigen::Vector3d rh);  // constructor

 private:
  Model model_;
  double dt_;
};
