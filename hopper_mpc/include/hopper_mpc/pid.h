#pragma once
#include <memory>
#include "Eigen/Dense"

class PID1 {  // The class

 public:                                             // Access specifier
  PID1(double dt, double kp, double ki, double kd);  // constructor
  double PIDControl(double input, double setp);

 private:
  double dt_;
  double err_;
  double err_sum_;
  double inp_prev_;
  double inp_diff_;
  double u_;
  double kp_;
  double ki_;
  double kd_;
};

class PID3 {  // The class

 public:                                                                        // Access specifier
  PID3(double dt, Eigen::Vector3d kp, Eigen::Vector3d ki, Eigen::Vector3d kd);  // constructor
  Eigen::Vector3d PIDControl(Eigen::Vector3d input, Eigen::Vector3d setp);
  Eigen::Vector3d PIDControlWrapped(Eigen::Vector3d input, Eigen::Vector3d setp);

 private:
  double dt_;
  Eigen::Vector3d err_;
  Eigen::Vector3d err_sum_;
  Eigen::Vector3d inp_prev_;
  Eigen::Vector3d inp_diff_;
  Eigen::Vector3d u_;

  Eigen::DiagonalMatrix<double, 3> kp_diag_;
  Eigen::DiagonalMatrix<double, 3> ki_diag_;
  Eigen::DiagonalMatrix<double, 3> kd_diag_;
};