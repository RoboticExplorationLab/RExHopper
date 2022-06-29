#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/bridge_mujoco.h"
#include "hopper_mpc/model.h"

class Leg {                             // The class
 public:                                // Access specifier
  Leg(Model model, float dt, float g);  // constructor
  Eigen::Vector4d q;
  Eigen::Vector4d dq;
  Eigen::Vector2d qa;
  Eigen::Vector2d dqa;
  Eigen::Vector4d q_prev;

  void UpdateState(Eigen::Vector2d a_in, Eigen::Quaterniond Q_base);

  Eigen::Vector2d KinInv(Eigen::Vector3d p_ref);
  Eigen::Vector3d KinFwd();
  Eigen::Vector3d GetVel();
  Eigen::Matrix<double, 4, 4> M;
  Eigen::Matrix<double, 4, 1> C;
  Eigen::Matrix<double, 4, 1> G;
  Eigen::Matrix<double, 3, 2> Ja;
  Eigen::Matrix3d Mx;

  void UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd);
  Eigen::Vector2d OpSpacePosCtrl(Eigen::Vector3d p_ref, Eigen::Vector3d v_ref);
  Eigen::Vector2d OpSpaceForceCtrl(Eigen::Vector3d f);
  Eigen::Vector2d InvKinPosCtrl(Eigen::Vector3d p_ref, float kp, float kd);

 private:
  Eigen::Vector2d a_cal_;

  Model model_;
  float dt_;
  Eigen::Matrix4d S_b_;      // actuator selection matrix (just the legs)
  Eigen::Vector3d gb_;       // 3D gravity vector in the body frame
  Eigen::Vector3d gb_init_;  // initial 3D gravity vector in the body frame, should not change
  double L0;
  double L1;
  double L2;
  double L3;
  double L4;
  double L5;
  void GenMCG();
  void GenJac();
  void GenMx();
  const double singularity_thresh_ = 0.00025;
  Eigen::Matrix3d Mx_inv_;
  Eigen::Matrix<double, 3, 4> J_;
  Eigen::DiagonalMatrix<double, 3> kp_diag_;
  Eigen::DiagonalMatrix<double, 3> kd_diag_;
  Eigen::Vector3d K_;
  int s_pol_;
  Eigen::Vector2d tau_;
};
