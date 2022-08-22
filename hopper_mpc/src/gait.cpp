#include "hopper_mpc/gait.h"
#include "hopper_mpc/utils.hpp"

Gait::Gait(Model model_, double dt_, std::shared_ptr<Leg>* legPtr_, std::shared_ptr<Rwa>* rwaPtr_) {
  model = model_;
  Eigen::Vector3d kp;
  Eigen::Vector3d ki;
  Eigen::Vector3d kd;
  kp << 0.02, 0.08, 0;
  ki << 0.2, 0.2, 0;
  kd << 0.01, 0.02, 0;
  pid_vPtr.reset(new PID3(dt, kp, ki, kd));
  legPtr = *legPtr_;
  rwaPtr = *rwaPtr_;
  z_ref = 0;
  pe_ref << 0, 0, model.h0;
  pf_ref.setZero();
}

Eigen::Matrix<double, 5, 1> Gait::uRaibert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q,
                                           Eigen::Vector3d v, Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref,
                                           Eigen::Vector3d v_ref, Eigen::Vector3d w_ref) {
  // continuous raibert hopping
  double z = 2 * asin(Q.z());  // z-axis of body quaternion
  Q_z.w() = cos(z / 2);
  Q_z.x() = 0;
  Q_z.y() = 0;
  Q_z.z() = sin(z / 2);
  R_z = (Q_z.inverse()).matrix();                                      // rotation matrix for body->world z turn
  v_ref = -Q_z.matrix() * pid_vPtr->PIDControl(R_z * p, R_z * p_ref);  // adjust for yaw, rotate back world->body
  double dist = (p_ref.block<2, 1>(0, 0) - p.block<2, 1>(0, 0)).norm();

  if (dist >= 1) {
    v_ref = p_ref - p;
    z_ref = atan2(v_ref(1), v_ref(0));
  }

  double k_b = (Utils::Clip(dist, 0.5, 1) + 2) / 3;
  double h = model.h0 * k_b;
  double kr = 0.15 / k_b;                                  // speed cancellation constant
  double kt = 0.4;                                         // leap period gain
  if (state == "Rise") {                                   // in first timestep after liftoff,
    if (state_prev == "Push") {                            // find new footstep position based on des and current speed
      pf_ref = v * kt * abs(v(2)) / 2 + kr * (v - v_ref);  // footstep in world frame for neutral motion + des acc
      pf_ref(2) = 0;                                       // enforce footstep is on ground plane
    }
    pe_ref(2) = h;  // pull leg up to prevent stubbing
  } else if (state == "Fall") {
    pe_ref(2) = h * 5.5 / 3;  // brace for impact
  } else if (state == "Cmpr") {
    pe_ref(2) = h * 4.5 / 3;  // TODO: Try reducing gains for compression instead of changing position setpoint
  } else if (state == "Push") {
    pe_ref(2) = h * 5.5 / 3;  // pushoff
  }

  Q_ref.setFromTwoVectors(pf_ref, p);  // Q_ref = utils.Q_inv(utils.vec_to_quat(self.x_des - p))
  u.block<2, 1>(0, 0) = legPtr->KinInvPosCtrl(pe_ref);
  u.block<3, 1>(2, 0) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  return u;
}

Eigen::Matrix<double, 5, 1> Gait::uKinInvVert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q,
                                              Eigen::Vector3d v, Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref,
                                              Eigen::Vector3d v_ref, Eigen::Vector3d w_ref) {
  Q_ref.coeffs() << 0, 0, 0, 1;  // xyzw format
  z_ref = 0;
  if (state == "Rise") {
    pe_ref(2) = model.h0;  // pull leg up to prevent stubbing
  } else if (state == "Fall") {
    pe_ref(2) = model.h0 * 5.5 / 3;  // brace for impact
  } else if (state == "Cmpr") {
    pe_ref(2) = model.h0 * 4.5 / 3;  // TODO: Try reducing gains for compression instead of changing position setpoint
  } else if (state == "Push") {
    pe_ref(2) = model.h0 * 5.5 / 3;  // pushoff
  }
  u.block<2, 1>(0, 0) = legPtr->KinInvPosCtrl(pe_ref);
  u.block<3, 1>(2, 0) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  return u;
}

Eigen::Matrix<double, 5, 1> Gait::uKinInvStand(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q,
                                               Eigen::Vector3d v, Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref,
                                               Eigen::Vector3d v_ref, Eigen::Vector3d w_ref) {
  Q_ref.coeffs() << 0, 0, 0, 1;  // xyzw format
  z_ref = 0;
  pe_ref(2) = model.h0 * 5 / 3;
  u.block<2, 1>(0, 0) = legPtr->KinInvPosCtrl(pe_ref);
  u.block<3, 1>(2, 0) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  return u;
}