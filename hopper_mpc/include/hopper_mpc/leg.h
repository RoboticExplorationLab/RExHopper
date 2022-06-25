#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/bridge_mujoco.h"
#include "hopper_mpc/model.h"

class Leg {          // The class
 public:             // Access specifier
  Leg(Model model);  // constructor
  Eigen::Vector4d q;
  Eigen::Vector4d q_prev;
  Eigen::Vector4d dq;
  void UpdateState(Eigen::Vector2d a_in, Eigen::Quaterniond Q_base);
  Eigen::Vector2d Leg::KinInv(Eigen::Vector3d p_ref);
  Eigen::Vector3d Leg::KinFwd();

 private:
  Eigen::Vector2d a_cal_;
  Model model_;
  float dt_;
  Eigen::Matrix4d S_b_;  // actuator selection matrix (just the legs)
  Eigen::Vector3d G_;
};
