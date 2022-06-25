#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/bridge_mujoco.h"
#include "hopper_mpc/model.h"

class Control {          // The class
 public:                 // Access specifier
  Control(Model model);  // constructor
  void Control::UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd);
  Eigen::Vector2d Control::OpSpacePosCtrl(Eigen::Vector3d p_ref, Eigen::Vector3d v_ref);
  Eigen::Vector2d Control::OpSpaceForceCtrl(Eigen::Vector3d f);
  Eigen::Vector2d Control::InvKinPosCtrl(Eigen::Vector3d p_ref, float kp, float kd);

 private:
  Eigen::DiagonalMatrix<float, 3> kp_diag_;
  Eigen::DiagonalMatrix<float, 3> kd_diag_;
  Eigen::Vector3d K_;
  int s_pol_;
  Eigen::MatrixXd S_b_;
  Eigen::VectorXd u_;
};
