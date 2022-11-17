#include "hopper_mpc/gait.h"
#include <iostream>
#include "hopper_mpc/utils.hpp"

Gait::Gait(Model model_, double dt_, Eigen::Vector3d peb_ref_, std::shared_ptr<Leg>* legPtr_, std::shared_ptr<Rwa>* rwaPtr_) {
  model = model_;
  dt = dt_;
  Eigen::Vector3d kp;
  Eigen::Vector3d ki;
  Eigen::Vector3d kd;
  kp << 0.02, 0.08, 0;
  ki << 0.02, 0.02, 0;
  kd << 0.01, 0.02, 0;
  pid_vPtr.reset(new PID3(dt, kp, ki, kd));
  legPtr = *legPtr_;
  rwaPtr = *rwaPtr_;
  z_ref = 0;
  peb_ref = peb_ref_;
  pf_ref.setZero();
  // x_adj = -0.002938125;  //-0.002938125 falls forward;  //-0.0029346875 falls backward;  works for rev08;
  x_adj = 0.00996875;  // this can be improved
  peb_ref << x_adj, 0, -model.h0 * 1.5;
}

uVals Gait::uRaibert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q, Eigen::Vector3d v,
                     Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref, Eigen::Vector3d v_ref, Eigen::Vector3d w_ref) {
  // continuous raibert hopping
  Eigen::Quaterniond Q_up;
  Q_up.w() = cos(1 / 2);
  Q_up.x() = 0;
  Q_up.y() = sin(1 / 2);
  Q_up.z() = 0;

  double z = 2 * asin(Q.z());  // z-axis of body quaternion
  Q_z.w() = cos(z / 2);
  Q_z.x() = 0;
  Q_z.y() = 0;
  Q_z.z() = sin(z / 2);
  R_z = (Q_z.inverse()).matrix();                                        // rotation matrix for body->world z turn
  v_ref = Q_z.matrix() * (-pid_vPtr->PIDControl(R_z * p, R_z * p_ref));  // adjust for yaw, rotate back world->body
  double dist = (p_ref.block<2, 1>(0, 0) - p.block<2, 1>(0, 0)).norm();
  // if (dist >= 0.2) {
  //   z_ref = atan2((p_ref - p)(1), (p_ref - p)(0));
  // }
  z_ref = atan2((p_ref - p)(1), (p_ref - p)(0));
  // double k_b = (Utils::Clip(dist, 0.5, 1) + 2) / 3;  // "braking" gain based on distance
  double h = model.h0;         // * k_b;                         // make jumps scale with distance to target
  double h_extend = h * 1.75;  // extension height
  double h_cmprss = h * 1.5;   // compression height

  if (state == "Rise") {                                               // in first timestep after liftoff,
    if (state_prev == "Push") {                                        // find new footstep position based on des and current speed
      double zeta = 2 * h_extend * sin(Utils::AngleBetween(Q, Q_up));  // add distance for leg location
      double kt = abs(2 * v(2) / 9.81);                                // leap period gain
      // double kr = 0.2 / k_b;                                           // desired acceleration constant
      double kr = 0.3;
      pf_ref = p + v * kt + kr * (v - v_ref) + v.normalized() * zeta;  // footstep in world frame for neutral motion + des acc + leg travel
      pf_ref(2) = 0;                                                   // enforce footstep is on ground plane
    }
    peb_ref << x_adj, 0, -h_cmprss;  // pull leg up to prevent stubbing
  } else if (state == "Fall") {
    // pew_ref = pf_ref - p;  // ref foot vector in world coordinates
    peb_ref = (Q.inverse().matrix() * (pf_ref - p)).normalized() * h_extend;
    // peb_ref << x_adj, 0, -h * 1.83;  // brace for impact
  } else if (state == "Cmpr") {
    peb_ref << x_adj, 0, -h_cmprss;  // TODO: Try reducing gains for compression instead of changing position setpoint
  } else if (state == "Push") {
    peb_ref << x_adj, 0, -h_extend;  // pushoff
  }
  Eigen::Vector3d v1;
  v1 << 0, 0, -1;  // datum vector, chosen as aligned with z-axis (representing leg direction)
  // Q_ref.setFromTwoVectors(v1, pf_ref - p);
  Q_ref = Utils::VecToQuat(pf_ref - p);
  Eigen::Vector2d qla_ref;
  qla_ref = legPtr->KinInv(peb_ref);
  std::string ctrlMode = "Pos";
  u.segment<3>(2) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::uKinInvVert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q, Eigen::Vector3d v,
                        Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref, Eigen::Vector3d v_ref, Eigen::Vector3d w_ref) {
  Q_ref.coeffs() << 0, 0, 0, 1;  // xyzw format
  z_ref = 0;
  if (state == "Rise") {
    peb_ref(2) = -model.h0;  // pull leg up to prevent stubbing
  } else if (state == "Fall") {
    peb_ref(2) = -model.h0 * 2;  // brace for impact
  } else if (state == "Cmpr") {
    peb_ref(2) = -model.h0 * 1.5;  // TODO: Try reducing gains for compression instead of changing position setpoint
  } else if (state == "Push") {
    peb_ref(2) = -model.h0 * 2;  // pushoff
  }
  std::string ctrlMode = "Torque";
  Eigen::Vector3d veb_ref;
  veb_ref.setZero();
  u.block<2, 1>(0, 0) = legPtr->OpSpacePosCtrl(peb_ref, veb_ref);
  Eigen::Vector2d qla_ref;
  qla_ref.setZero();
  // Eigen::Vector2d qla_ref = legPtr->KinInv(peb_ref);
  // std::string ctrlMode = "Pos";
  u.segment<3>(2) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::uKinInvStand(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q, Eigen::Vector3d v,
                         Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref, Eigen::Vector3d v_ref, Eigen::Vector3d w_ref) {
  Q_ref.coeffs() << 0, 0, 0, 1;  // xyzw format
  z_ref = 0;
  peb_ref(2) = -model.h0 * 5 / 3;
  // double kp = model.k_kin(0) * 2;
  // double kd = model.k_kin(1) * 2;
  // u.block<2, 1>(0, 0) = legPtr->KinInvPosCtrl(peb_ref, kp, kd);
  Eigen::Vector2d qla_ref;
  qla_ref = legPtr->KinInv(peb_ref);
  std::string ctrlMode = "Pos";
  u.segment<3>(2) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  // u = u.cwiseQuotient(model.a_kt);  // u = u ./ model.a_kt
  return uVals{u, qla_ref, ctrlMode};
}