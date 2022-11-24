#include "hopper_mpc/observer.h"
#include "Eigen/Dense"

// #include "hopper_mpc/utils.hpp"

Observer::Observer(double dt_, std::shared_ptr<Leg>* legPtr_) {
  dt = dt_;
  legPtr = *legPtr_;

  delay_term.setZero();

  double lambda = 15;  // cutoff frequency. TODO: Tune
  gamma = exp(-lambda * dt);
  beta = (1 - gamma) / (gamma * dt);
};  // constructor

Eigen::Vector4d Observer::TorqueEst(Eigen::Vector2d u) {
  /* Generalized momentum-based discrete time filtered disturbance torque observer
  Isolates disturbance torque from other sources
  From:
  Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains
  Gerardo Bledt, Patrick M. Wensing, Sam Ingersoll, and Sangbae Kim */

  Eigen::Vector4d tau_leg;
  tau_leg << u(0), 0, u(1), 0;
  Eigen::Vector4d p = legPtr->M * legPtr->dq;  // generalized momentum
  // TODO: Add coriolis term?
  Eigen::Vector4d tau_dist = beta * p - (1 - gamma) * (beta * p + tau_leg + legPtr->G) + gamma * delay_term;

  delay_term = tau_dist - beta * p;

  return tau_dist;
};

// Eigen::Vector3d Observer::ForceEst(Eigen::Vector2d u) {
//   // tau = J^T * F
//   // F = (J^T)^-1 * tau
//   Eigen::Matrix<double, 4, 3> pinv = (legPtr->J).transpose().completeOrthogonalDecomposition().pseudoInverse();

//   return pinv * TorqueEst(u);
// }