#include "hopper_mpc/ctrl.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/model.h"

Control::Control(Model model) {
  // Take in body frame positions or forces for the end effector and translate to joint torques
  u_.setZero(model.n_a);
  s_pol_ = model.s_pol;
  S_b_ = model.S.block<4, 4>(0, 0);  // actuator selection matrix (just the legs)
  K_ << model.K, model.K, model.K;
  UpdateGains(K_, K_ * 0.02);
}

void Control::UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd) {
  // Use this to update wbc PD gains in real time
  kp_diag_ = kp.asDiagonal();
  kd_diag_ = kd.asDiagonal();
}

Eigen::Vector2d Control::OpSpacePosCtrl(Eigen::Vector3d p_ref, Eigen::Vector3d v_ref) {
  // x = leg.position()
  // Ja = leg.gen_jacA()  // 3x2
  // vel = leg.velocity()
  // x_dd_des = np.dot(kp, (target - x)) + np.dot(kd, (target_vel - vel))  // .reshape((-1, 1))
  // Mx = leg.gen_Mx()
  // fx = Mx @ x_dd_des
  // tau = Ja.T @ fx
  // u = tau.flatten() - spring.fn_spring(leg.q[0], leg.q[2])
  return -u;
}

Eigen::Vector2d Control::OpSpaceForceCtrl(Eigen::Vector3d f) {
  // leg = leg
  // Ja = leg.gen_jacA()  // 3x2
  // u = (Ja.T @ force).flatten() - spring.fn_spring(leg.q[0], leg.q[2])
  return -u;
}

Eigen::Vector2d Control::InvKinPosCtrl(Eigen::Vector3d p_ref, float kp, float kd) {
  // leg = leg
  // dqa = np.array([leg.dq[0], leg.dq[2]])
  // qa = np.array([leg.q[0], leg.q[2]])
  // u = (qa - leg.inv_kinematics(xyz=target[0:3])) * kp + dqa * kd - spring.fn_spring(leg.q[0], leg.q[2])
  return u;
}