#include "hopper_mpc/actuator.h"
#include <algorithm>
#include "Eigen/Core"

Actuator::Actuator(ActuatorModel a_model_, double dt_) {
  a_model = a_model_;
  dt = dt_;
  v_max = a_model.v_max;  // omega_max * self.kt  # absolute maximum
  gr = a_model.gr;
  i_max = a_model.i_max;
  r = a_model.r;
  kt = a_model.kt;
  tau_max = i_max * kt;
  omega_max = v_max / kt;

  // smoothing bandwidth
  i_smoothed = 0;
  double tau = 1 / 160;  // inverse of Odrive torque bandwidth

  // G(s) = tau/(s+tau)  http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
  alpha = dt / (dt + tau);  // #DT version of low pass filter
};

OutVals Actuator::Actuate(double tau_ref, double dq) {
  // simulates actuator physics by limiting torque output as a function of motor speed
  double i = tau_ref / kt;
  double omega = dq * gr;  // motor speed is joint speed times gear ratio
  i_smoothed = (1 - alpha) * i_smoothed + alpha * i;
  i = i_smoothed;

  double v = sgn(i) * v_max;
  double tau_ul = (-omega * pow(kt, 2) + v * kt) / r;  // max motor torque for given speed
  double tau_ll = (-omega * pow(kt, 2) - v * kt) / r;  // min motor torque for given speed
  double tau_m = kt * i;
  if (tau_ul >= tau_ll) {
    tau_m = clip(tau_m, tau_ll, tau_ul);
  } else {
    tau_m = clip(tau_m, tau_ul, tau_ll);
  }
  tau_m = clip(tau_m, -tau_max, tau_max);
  i = abs(tau_m / kt);
  v = abs(i * r + kt * clip(omega, -omega_max, omega_max));
  double tau = tau_m * gr;
  return OutVals{tau, i, v};
}

template <typename T>
int Actuator::sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T>
T Actuator::clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}
