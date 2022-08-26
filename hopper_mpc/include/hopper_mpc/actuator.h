#pragma once

#include "hopper_mpc/model.h"

class Actuator {                                 // The class
 public:                                         // Access specifier
  Actuator(ActuatorModel a_model_, double dt_);  // constructor
  OutVals Actuate(double tau_ref, double dq);

 private:
  double dt;
  ActuatorModel a_model;
  double v_max;
  double gr;
  double i_max;
  double r;
  double kt;
  double tau_max;
  double omega_max;
  double i_smoothed;  // smoothing bandwidth
  double alpha;

  template <typename T>
  int sgn(T val);

  template <typename T>
  T clip(const T& n, const T& lower, const T& upper);
};
