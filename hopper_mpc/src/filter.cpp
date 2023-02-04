#include "hopper_mpc/filter.h"
#include <iostream>
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

Notch::Notch(double dt_, double f_, double BW_) {
  // https://dsp.stackexchange.com/a/11291
  // http://dspguide.com/ch19/3.htm
  // f_   // center frequency as a fraction of the sampling frequency, must be between 0 and 0.5.
  // BW_  // bandwidth as a fraction of the sampling frequency, must be between 0 and 0.5.
  assert(f_ > 0 && f_ < 0.5);
  assert(BW_ > 0 && BW_ < 0.5);

  double costerm = cos(2 * M_PI * f_);
  // intermediate variables
  double R = 1 - 3 * BW_;
  double K = (1 - 2 * R * costerm + pow(R, 2)) / (2 - 2 * costerm);
  // recursion coeffs
  a0 = 1 - K;
  a1 = -2 * K * costerm;
  a2 = K;
  b1 = 2 * R * costerm;
  b2 = -(R * R);

  // init
  x1 = 0.0;
  x2 = 0.0;
  y1 = 0.0;
  y2 = 0.0;
}

double Notch::Filter(double x0) {
  double y0 = a0 * x0 + a1 * x1 + a2 * x2  // IIR difference equation
              + b1 * y1 + b2 * y2;
  x2 = x1;  // shift delayed x, y samples
  x1 = x0;
  y2 = y1;
  y1 = y0;
  return y1;
}

// double fs = 500;       // sampling rate of imu
// double fn = fs / 2.0;  // nyquist frequency
// notchWidth = 0.1;      // width of the notch

// // Compute zeros
// notchZeros = [ exp(sqrt(-1) * pi * freqRatio), exp(-sqrt(-1) * pi * freqRatio) ];

// // Compute poles
// notchPoles = (1 - notchWidth) * notchZeros;

// b = poly(notchZeros);
// // Get moving average filter coefficients
// a = poly(notchPoles);
// // Get autoregressive filter coefficients

// freqz(b, a, 32000, fs);

// // filter signal x
// y = filter(b, a, x);