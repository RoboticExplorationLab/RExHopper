#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/model.h"

class Leg {                       // The class
 public:                          // Access specifier
  Leg(Model model_, double dt_);  // constructor
  Eigen::Vector4d q;
  Eigen::Vector4d dq;
  Eigen::Vector2d qa;   // just the actuated leg joint pos (needed for jacobians)
  Eigen::Vector2d dqa;  // just the actuated leg joint vel (needed for jacobians)
  Eigen::Vector4d q_prev;

  void UpdateState(Eigen::Vector2d q_in, Eigen::Quaterniond Q_base);

  Eigen::Vector2d KinInv(Eigen::Vector3d p_ref);
  Eigen::Vector3d GetPos();
  Eigen::Vector3d KinFwd(double q0, double q2);

  Eigen::Vector3d GetVel();
  void UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd);
  Eigen::Vector2d OpSpacePosCtrl(Eigen::Vector3d p_ref, Eigen::Vector3d v_ref);
  Eigen::Vector2d OpSpaceForceCtrl(Eigen::Vector3d f);
  Eigen::Vector2d KinInvPosCtrl(Eigen::Vector3d p_ref, double kp, double kd);

  Eigen::Matrix<double, 4, 4> M;
  Eigen::Matrix<double, 4, 1> C;
  Eigen::Matrix<double, 4, 1> G;
  Eigen::Matrix<double, 3, 2> Ja;
  Eigen::Matrix3d Mx;
  Eigen::Matrix<double, 3, 4> J;

 private:
  void GenMCG();
  void GenJac();
  void GenMx();
  Eigen::Matrix<double, 2, 1> qa_out;

  Model model;
  double dt;
  Eigen::Matrix4d S_b;      // actuator selection matrix (just the legs)
  Eigen::Vector3d gb;       // 3D gravity vector in the body frame
  Eigen::Vector3d gb_init;  // initial 3D gravity vector in the body frame, should not change
  double L0;
  double L1;
  double L2;
  double L3;
  double L4;
  double L5;

  const double singularity_thresh = 0.00025;
  Eigen::Matrix3d Mx_inv;

  Eigen::DiagonalMatrix<double, 3> kp_diag;
  Eigen::DiagonalMatrix<double, 3> kd_diag;
  Eigen::Vector3d K;
  int s_pol;
  Eigen::Vector2d tau;
};
