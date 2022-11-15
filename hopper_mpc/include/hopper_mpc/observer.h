#pragma once
#include "Eigen/Dense"

class Observer {  // The class

 public:                 // Access specifier
  Observer(double dt_);  // constructor
  Eigen::Vector4d TorqueEst(Eigen::Matrix4d Mq, Eigen::Vector4d dq, Eigen::Vector4d tau_leg, Eigen::Vector4d tau_grav);

 private:
  double dt;
  double gamma;
  double beta;
  Eigen::Vector4d delay_term;
  Eigen::Vector4d tau_dist;
};
