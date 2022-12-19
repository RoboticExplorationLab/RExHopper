#include "hopper_mpc/actuator.h"
#include <algorithm>
#include <iostream>
#include "Eigen/Core"
#include "hopper_mpc/utils.hpp"

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
  lowpassPtr.reset(new LowPass(dt, 160));  // 160 = Odrive torque bandwidth
};

OutVals Actuator::Actuate(double tau_ref, double dq) {
  // simulates actuator physics by limiting torque output as a function of motor speed
  double i = tau_ref / kt;
  double omega = dq * gr;  // motor speed is joint speed times gear ratio

  i = lowpassPtr->Filter(i);  // smooth based on odrive bandwidth
  double tau_m = kt * i;
  double v = copysign(1.0, i) * v_max;
  double tau_ul = (-omega * pow(kt, 2) + v * kt) / r;  // max motor torque for given speed
  double tau_ll = (-omega * pow(kt, 2) - v * kt) / r;  // min motor torque for given speed
  // std::cout << "v_max, v, omega, ul, ll = " << v_max << ", " << v << ", " << omega << ", " << tau_ul << ", " << tau_ll << "\n";
  if (tau_ul >= tau_ll) {
    tau_m = Utils::Clip(tau_m, tau_ll, tau_ul);
  } else {
    tau_m = Utils::Clip(tau_m, tau_ul, tau_ll);
  }
  // std::cout << "tau_m = " << tau_m << "\n";
  tau_m = Utils::Clip(tau_m, -tau_max, tau_max);
  i = abs(tau_m / kt);
  v = abs(i * r + kt * Utils::Clip(omega, -omega_max, omega_max));
  double tau = tau_m * gr;
  return OutVals{tau, i, v};
}
