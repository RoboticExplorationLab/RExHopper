#include "hopper_mpc/observer.h"
#include "Eigen/Dense"
// #include "hopper_mpc/utils.hpp"

Observer::Observer(double dt_) {
  dt = dt_;
  delay_term.setZero();
  tau_dist.setZero();

  double lambda = 15;  // cutoff frequency. TODO: Tune
  gamma = exp(-lambda * dt);
  beta = (1 - gamma) / (gamma * dt);
};  // constructor

Eigen::Vector4d Observer::TorqueEst(Eigen::Matrix4d Mq, Eigen::Vector4d dq, Eigen::Vector4d tau_leg, Eigen::Vector4d tau_grav) {
  /* Generalized momentum-based discrete time filtered disturbance torque observer
  Isolates disturbance torque from other sources
  From:
  Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains
  Gerardo Bledt, Patrick M. Wensing, Sam Ingersoll, and Sangbae Kim */
  Eigen::Vector4d p = Mq * dq;  // generalized momentum
  // TODO: Add coriolis term?
  tau_dist = beta * p - (1 - gamma) * (beta * p + tau_leg + tau_grav) + gamma * delay_term;
  delay_term = tau_dist - beta * p;
  return tau_dist;
};