#include "hopper_mpc/rwa.h"
#include <iostream>
#include "Eigen/Core"
#include "hopper_mpc/utils.hpp"

Rwa::Rwa(double dt_) {
  // q = model.init_q;
  // q_prev = q;
  // dq = model.init_dq;

  dt = dt_;

  a = 45 * M_PI / 180;
  b = -45 * M_PI / 180;
  sin45 = sin(45 * M_PI / 180);

  double ku = 200;  // 200 TODO: Might want to increase this.
  // use gain of 13 for CoM bisection search.
  // can go as high as 1300 (not sure if necessary)
  kp_tau << ku * 0.6, ku * 0.6, ku * 0.5 * 0.6;
  ki_tau << ku * 0.56, ku * 0.56, ku * 0.5 * 0.56;        // ki_tau << ku * 0.1, ku * 0.1, ku * 0.01;
  kd_tau << ku * 0.1875, ku * 0.1875, ku * 0.1875 * 0.5;  // ku * 0.04, ku * 0.04, ku * 0.005;

  // PID3 pid_tau(dt, kp_tau, ki_tau, kd_tau);
  pid_tauPtr.reset(new PID3(dt, kp_tau, ki_tau, kd_tau));
  double ks = 0;  // 0.001;  // 2;
  kp_vel << ks, ks, ks;
  ki_vel << ks * 0.1, ks * 0.1, ks * 0.1;
  kd_vel << 0, 0, 0;
  pid_velPtr.reset(new PID3(dt, kp_vel, ki_vel, kd_vel));
};

// void Rwa::UpdateState(Eigen::Vector3d q_in) {
//   // Pull raw actuator joint values in from simulator or robot
//   // Make sure this only happens once per time step
//   q = q_in;
//   dq = (q - q_prev) / dt_;
//   q_prev = q;
//   // dq_prev = dq;
// };

double Rwa::GetXRotatedAboutZ(Eigen::Quaterniond Q_in, double z) {
  // rotate quaternion about its z - axis by specified angle "z"
  // and get rotation about x-axis of that (confusing, I know)
  Eigen::Quaterniond Q_z;
  Q_z.w() = cos(z / 2);
  Q_z.x() = 0;
  Q_z.y() = 0;
  Q_z.z() = sin(z / 2);
  Eigen::Quaterniond Q_res;
  Q_res = Q_z * Q_in;  // Q_res = L(Q_z).T @ Q_in
  Q_res.normalize();   // TODO: Is this necessary or is it automatic?
  return 2 * asin(Q_res.x());
}

Eigen::Vector3d Rwa::AttitudeIn(Eigen::Quaterniond Q_ref, Eigen::Quaterniond Q_base) {
  // get body angle in rw axes
  theta(0) = GetXRotatedAboutZ(Q_base, a);
  theta(1) = GetXRotatedAboutZ(Q_base, b);
  theta(2) = 2 * asin(Q_base.z());  // z-axis of body quaternion
  return theta;
}

Eigen::Vector3d Rwa::AttitudeSetp(Eigen::Quaterniond Q_ref, double z_ref) {
  // get setpoint in rw axes
  Eigen::Vector3d setp;
  double adj = 0;  // -0.34652 * M_PI / 180;
  setp(0) = GetXRotatedAboutZ(Q_ref, a) + adj;
  setp(1) = GetXRotatedAboutZ(Q_ref, b) - adj;
  setp(2) = z_ref;
  return setp;
}

Eigen::Vector3d Rwa::AttitudeCtrl(Eigen::Quaterniond Q_ref, Eigen::Quaterniond Q_base, double z_ref) {
  // simple reaction wheel attitude control w/ derivative on measurement pid
  theta = AttitudeIn(Q_ref, Q_base);
  setp = AttitudeSetp(Q_ref, z_ref);
  setp = setp - pid_velPtr->PIDControl(theta, setp);
  return pid_tauPtr->PIDControlWrapped(theta, setp);  // Cascaded PID Loop
}

Eigen::Vector3d Rwa::TorqueCtrl(Eigen::Vector3d tau_ref) {
  // simple reaction wheel torque control (rotated into rw axes)
  Eigen::Vector3d tau;
  tau(0) = (tau_ref(0) - tau_ref(1)) / (2 * sin45);
  tau(1) = (tau_ref(0) - tau_ref(1)) / (2 * sin45);
  tau(2) = tau_ref(2);
  return tau;
}