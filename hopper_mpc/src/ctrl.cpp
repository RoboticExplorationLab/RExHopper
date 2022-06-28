#include "hopper_mpc/ctrl.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"

Control::Control(Model model, Leg& leg) {
  // Take in body frame positions or forces for the end effector and translate to joint torques
  s_pol_ = model.s_pol;
  S_b_ = model.S.block<4, 4>(0, 0);  // actuator selection matrix (just the legs)
  K_ << model.K, model.K, model.K;
  UpdateGains(K_, K_ * 0.02);
  legPtr_* = leg;
}

void Control::UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd) {
  // Use this to update wbc PD gains in real time
  kp_diag_ = kp.asDiagonal();
  kd_diag_ = kd.asDiagonal();
}

Eigen::Vector2d Control::OpSpacePosCtrl(Eigen::Vector3d p_ref, Eigen::Vector3d v_ref) {
  Eigen::Vector3d p;
  p = legPtr_->KinFwd();
  Eigen::Matrix<double, 3, 2> Ja;
  Ja = legPtr_->Ja;
  Eigen::Vector3d v;
  v = legPtr_->GetVel();
  Eigen::Vector3d pdd_ref;
  pdd_ref = kp_diag_ * (p_ref - p) + kd_diag_ * (v_ref - v);
  Eigen::Vector3d f;
  f = (legPtr_->Mx) * pdd_ref;
  tau_ = Ja.transpose() * f;  // u = tau.flatten() - spring.fn_spring(leg.q[0], leg.q[2])
  return -tau_;
}

Eigen::Vector2d Control::OpSpaceForceCtrl(Eigen::Vector3d f) {
  Eigen::Matrix<double, 3, 2> Ja;
  Ja = legPtr_->Ja;
  tau_ = Ja.transpose() * f;  // u = (Ja.T @ force).flatten() - spring.fn_spring(leg.q[0], leg.q[2])
  return -tau_;
}

Eigen::Vector2d Control::InvKinPosCtrl(Eigen::Vector3d p_ref, float kp, float kd) {
  tau_ = kp * (legPtr_->qa - legPtr_->KinInv(p_ref)) + kd * legPtr_->dqa;
  return tau_;
}