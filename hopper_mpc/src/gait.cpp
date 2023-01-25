#include "hopper_mpc/gait.h"
#include <math.h>
#include <iostream>
#include "hopper_mpc/utils.hpp"

Gait::Gait(Model model_, double dt_, Eigen::Vector3d peb_ref_, std::shared_ptr<Leg>* legPtr_, std::shared_ptr<Rwa>* rwaPtr_,
           double x_adj_) {
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

  x_adj = x_adj_;

  peb_ref << x_adj, 0, -model.h0 * 1.5;
  u.setZero();

  // standup
  N_getup = model.N_getup;
  peb_ref_init = legPtr->KinFwd(model.qla_sit(0), model.qla_sit(1));
  peb_ref_final << x_adj, 0, -model.h0 * 5 / 3;
  peb_ref_trajx = Eigen::VectorXd::LinSpaced(N_getup, peb_ref_init(0), peb_ref_final(0));
  peb_ref_trajz = Eigen::VectorXd::LinSpaced(N_getup, peb_ref_init(2), peb_ref_final(2));
}

uVals Gait::Raibert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q, Eigen::Vector3d v,
                    Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref, Eigen::Vector3d v_ref, Eigen::Vector3d w_ref) {
  // continuous raibert hopping
  Eigen::Quaterniond Q_up;
  Q_up.w() = cos(1 / 2);
  Q_up.x() = 0;
  Q_up.y() = sin(1 / 2);
  Q_up.z() = 0;
  Q_z = Utils::ExtractYawQuat(Q);
  R_z = (Q_z.conjugate()).matrix();                                      // rotation matrix for body->world z turn
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
      double kt = abs(2 * v(2) / model.g);                             // leap period gain
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
  Q_ref = Utils::VecToQuat(v1, pf_ref - p);
  Eigen::Vector2d qla_ref;
  qla_ref = legPtr->KinInv(peb_ref);
  std::string ctrlMode = "Pos";
  u.segment<3>(2) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::KinInvStand(Eigen::Quaterniond Q) {
  Eigen::Quaterniond Q_ref;
  Q_ref.setIdentity();
  z_ref = 0;
  // z_ref = 15 * M_PI / 180;
  peb_ref(2) = -model.h0 * 5.0 / 3.0;
  // double kp = model.k_kin(0) * 2;
  // double kd = model.k_kin(1) * 2;
  // u.block<2, 1>(0, 0) = legPtr->KinInvPosCtrl(peb_ref, kp, kd);
  Eigen::Vector2d qla_ref = legPtr->KinInv(peb_ref);
  std::string ctrlMode = "Pos";
  u.segment<3>(2) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::GetUp(Eigen::Quaterniond Q) {
  Eigen::Quaterniond Q_ref;
  Q_ref.setIdentity();
  z_ref = 0;
  peb_ref << peb_ref_trajx(i), 0.0, peb_ref_trajz(i);
  Eigen::Vector2d qla_ref = legPtr->KinInv(peb_ref);
  std::string ctrlMode = "Pos";
  u.segment<3>(2) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  i += 1;
  if (i >= N_getup) {
    i = N_getup - 1;
  }
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::Sit() {
  Eigen::Quaterniond Q_ref;
  Q_ref.setIdentity();
  z_ref = 0;
  Eigen::Vector2d qla_ref = model.qla_sit;
  std::string ctrlMode = "Pos";
  // u.segment<3>(2) = rwaPtr->AttitudeCtrl(Q_ref, Q, z_ref);
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::Idle() {
  Eigen::Vector2d qla_ref;
  qla_ref.setZero();
  std::string ctrlMode = "None";
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::CircleTest() {
  // edits peb_ref in-place
  z += 0.0005 * flip;  // std::cout << z << "\n";
  if (z <= -0.5 || z >= -0.3) {
    flip *= -1;
  }
  double x = (sqrt(pow(r, 2) - pow(z - z1, 2)) + x1) * -flip;
  if (isnan(x)) {  // uses math.h
    x = 0;
  }
  peb_ref(0) = x;
  peb_ref(2) = z;

  Eigen::Vector2d qla_ref;
  qla_ref = legPtr->KinInv(peb_ref);
  std::string ctrlMode = "Pos";
  // std::cout << "peb_ref = " << peb_ref(0) << ", " << peb_ref(1) << ", " << peb_ref(2) << "\n";
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::VelTest() {
  // check reaction wheel speed polarity matches torque polarity
  Eigen::Vector2d qla_ref;
  qla_ref.setZero();
  std::string ctrlMode = "None";  // legs go limp

  u.segment<3>(2) = rwaPtr->RotorVelCtrl();
  return uVals{u, qla_ref, ctrlMode};
}

uVals Gait::PosTest() {
  // check reaction wheel pos polarity matches torque polarity
  Eigen::Vector2d qla_ref;
  qla_ref.setZero();
  std::string ctrlMode = "None";  // legs go limp

  u.segment<3>(2) = rwaPtr->RotorPosCtrl();
  return uVals{u, qla_ref, ctrlMode};
}