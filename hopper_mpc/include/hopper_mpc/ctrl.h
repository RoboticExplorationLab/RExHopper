#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/bridge_mujoco.h"
#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"

class Control {                    // The class
 public:                           // Access specifier
  Control(Model model, Leg& leg);  // constructor
  void UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd);
  Eigen::Vector2d OpSpacePosCtrl(Eigen::Vector3d p_ref, Eigen::Vector3d v_ref);
  Eigen::Vector2d OpSpaceForceCtrl(Eigen::Vector3d f);
  Eigen::Vector2d InvKinPosCtrl(Eigen::Vector3d p_ref, float kp, float kd);

 private:
  Eigen::DiagonalMatrix<float, 3> kp_diag_;
  Eigen::DiagonalMatrix<float, 3> kd_diag_;
  Eigen::Vector3d K_;
  int s_pol_;
  Eigen::Matrix4d S_b_;
  Eigen::Vector3d tau_;
  std::unique_ptr<Leg> legPtr_;
};
