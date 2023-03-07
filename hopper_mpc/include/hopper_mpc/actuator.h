#pragma once

#include <memory>
#include "hopper_mpc/filter.h"
#include "hopper_mpc/model.h"

struct ActuatorModel {
  std::string name;
  double v_max;
  double kt;
  double omega_max;
  double tau_max;
  double r;
  double i_max;
  double gr;
};

struct OutVals {
  double tau_out;
  double i;
  double v;
};

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

  std::unique_ptr<LowPass> lowpassPtr;
};
