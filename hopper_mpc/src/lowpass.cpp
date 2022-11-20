#include "hopper_mpc/lowpass.h"

LowPass::LowPass(double dt_, double bandwidth_) {
  // smoothing bandwidth
  input_smoothed = 0;
  double tau = 1 / bandwidth_;  // 160 = Odrive torque bandwidth
  // G(s) = tau/(s+tau)  http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
  alpha = dt_ / (dt_ + tau);  // #DT version of low pass filter
}

double LowPass::Filter(double input) {
  input_smoothed = (1 - alpha) * input_smoothed + alpha * input;
  return input_smoothed;
}

LowPass3D::LowPass3D(double dt_, double bandwidth_) {
  // smoothing bandwidth
  input_smoothed.setZero();
  double tau = 1 / bandwidth_;  // 160 = Odrive torque bandwidth
  // G(s) = tau/(s+tau)  http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
  alpha = dt_ / (dt_ + tau);  // #DT version of low pass filter
}

Eigen::Vector3d LowPass3D::Filter(Eigen::Vector3d input) {
  input_smoothed = (1 - alpha) * input_smoothed + alpha * input;
  return input_smoothed;
}