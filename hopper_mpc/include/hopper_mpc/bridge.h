#pragma once

#include "hopper_mpc/model.h"

class Bridge {                                                  // The class
 public:                                                        // Access specifier
  Bridge(Model model_, double dt_, bool fixed_, bool record_);  // constructor
  virtual void Init() = 0;
  virtual retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) = 0;
  virtual void End() = 0;

 protected:
  Model model;
  double dt;
  double g;
  double mu;
  bool fixed;
  bool record;
  Eigen::Matrix<double, 5, 1> qa_cal;
  Eigen::Matrix<double, 5, 1> qa_raw;
  Eigen::Matrix<double, 5, 1> qa;
  Eigen::Matrix<double, 5, 1> dqa;
  Eigen::Matrix<double, 13, 1> X;
};
