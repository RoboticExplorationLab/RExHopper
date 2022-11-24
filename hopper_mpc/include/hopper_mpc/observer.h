#pragma once
#include <memory>  // for shared_ptr
#include "Eigen/Dense"
#include "hopper_mpc/leg.h"

class Observer {  // The class

 public:                                                // Access specifier
  Observer(double dt_, std::shared_ptr<Leg>* legPtr_);  // constructor
  Eigen::Vector4d TorqueEst(Eigen::Vector2d u);
  Eigen::Vector3d ForceEst(Eigen::Vector2d u);

 private:
  double dt;
  double gamma;
  double beta;
  Eigen::Vector4d delay_term;

  std::shared_ptr<Leg> legPtr;
};
