#pragma once

#include "hopper_mpc/model.h"

class Bridge {                                                                   // The class
 public:                                                                         // Access specifier
  Bridge(Model model, double dt, double g, double mu, bool fixed, bool record);  // constructor
  virtual void Init() = 0;
  virtual void SimRun(Eigen::Matrix<double, 5, 1> u) = 0;
  virtual void End() = 0;

 protected:
  Model model_;
  double dt_;
  double g_;
  double mu_;
  bool fixed_;
  bool record_;
};