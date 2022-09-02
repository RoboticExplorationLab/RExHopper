#pragma once
#include <memory>
#include "Eigen/Dense"

class PID1 {  // The class

 public:                                                 // Access specifier
  PID1(double dt_, double kp_, double ki_, double kd_);  // constructor
  double PIDControl(double input, double setp);

 private:
  double dt;
  double err;
  double err_sum;
  double inp_prev;
  double inp_diff;
  double u;
  double kp;
  double ki;
  double kd;
};

class PID3 {  // The class

 public:                                                                            // Access specifier
  PID3(double dt_, Eigen::Vector3d kp_, Eigen::Vector3d ki_, Eigen::Vector3d kd_);  // constructor
  Eigen::Vector3d PIDControl(Eigen::Vector3d input, Eigen::Vector3d setp);
  Eigen::Vector3d PIDControlWrapped(Eigen::Vector3d input, Eigen::Vector3d setp);

 private:
  double dt;
  Eigen::Vector3d err;
  Eigen::Vector3d err_sum;
  Eigen::Vector3d inp_prev;
  Eigen::Vector3d inp_diff;
  Eigen::Vector3d u;

  Eigen::DiagonalMatrix<double, 3> kp_diag;
  Eigen::DiagonalMatrix<double, 3> ki_diag;
  Eigen::DiagonalMatrix<double, 3> kd_diag;
};