#include "hopper_mpc/pid.h"
#include "Eigen/Dense"
#include "hopper_mpc/utils.hpp"

PID1::PID1(double dt, double kp, double ki, double kd) {
  dt_ = dt;
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  inp_prev_ = 0;
};  // constructor

double PID1::PIDControl(double input, double setp) {
  err_ = input - setp;
  inp_diff_ = (input - inp_prev_) / dt_;
  u_ = kp_ * err_ + ki_ * err_sum_ * dt_ + kd_ * inp_diff_;
  inp_prev_ = input;
  err_sum_ += err_;
  return u_;
};

PID3::PID3(double dt, Eigen::Vector3d kp, Eigen::Vector3d ki, Eigen::Vector3d kd) {
  dt_ = dt;
  kp_diag_ = kp.asDiagonal();
  ki_diag_ = ki.asDiagonal();
  kd_diag_ = kd.asDiagonal();
  inp_prev_.setZero();
};  // constructor

Eigen::Vector3d PID3::PIDControl(Eigen::Vector3d input, Eigen::Vector3d setp) {
  err_ = input - setp;
  inp_diff_ = (input - inp_prev_) / dt_;
  u_ = kp_diag_ * err_ + ki_diag_ * err_sum_ * dt_ + kd_diag_ * inp_diff_;
  inp_prev_ = input;
  err_sum_ += err_;
  return u_;
};

Eigen::Vector3d PID3::PIDControlWrapped(Eigen::Vector3d input, Eigen::Vector3d setp) {
  for (int i = 0; i < 3; i++) {
    err_(i) = Utils::WrapToPi(input(i) - setp(i));
  }
  inp_diff_ = (input - inp_prev_) / dt_;
  u_ = kp_diag_ * err_ + ki_diag_ * err_sum_ * dt_ + kd_diag_ * inp_diff_;
  inp_prev_ = input;
  err_sum_ += err_;
  return u_;
};