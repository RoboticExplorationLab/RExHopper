#include "hopper_mpc/pid.h"
#include "Eigen/Dense"
#include "hopper_mpc/utils.hpp"

PID1::PID1(double dt_, double kp_, double ki_, double kd_) {
  dt = dt_;
  kp = kp_;
  ki = ki_;
  kd = kd_;
  inp_prev = 0;
};  // constructor

double PID1::PIDControl(double input, double setp) {
  err = input - setp;
  inp_diff = (input - inp_prev) / dt;
  u = kp * err + ki * err_sum * dt + kd * inp_diff;
  inp_prev = input;
  err_sum += err;
  return u;
};

PID3::PID3(double dt_, Eigen::Vector3d kp_, Eigen::Vector3d ki_, Eigen::Vector3d kd_) {
  dt = dt_;
  kp_diag = kp_.asDiagonal();
  ki_diag = ki_.asDiagonal();
  kd_diag = kd_.asDiagonal();
  inp_prev.setZero();
};  // constructor

Eigen::Vector3d PID3::PIDControl(Eigen::Vector3d input, Eigen::Vector3d setp) {
  err = input - setp;
  inp_diff = (input - inp_prev) / dt;
  u = kp_diag * err + ki_diag * err_sum * dt + kd_diag * inp_diff;
  inp_prev = input;
  err_sum += err;
  return u;
};

Eigen::Vector3d PID3::PIDControlWrapped(Eigen::Vector3d input, Eigen::Vector3d setp) {
  for (int i = 0; i < 3; i++) {
    err(i) = Utils::WrapToPi(input(i) - setp(i));
  }
  inp_diff = (input - inp_prev) / dt;
  u = kp_diag * err + ki_diag * err_sum * dt + kd_diag * inp_diff;
  inp_prev = input;
  err_sum += err;
  return u;
};