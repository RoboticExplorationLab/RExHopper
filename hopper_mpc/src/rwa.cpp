#include "hopper_mpc/rwa.h"
#include "Eigen/Core"
#include "hopper_mpc/utils.hpp"

Rwa::Rwa(float dt) {
  // q = model.init_q;
  // q_prev = q;
  // dq = model.init_dq;

  dt_ = dt;

  a_ = -45 * M_PI / 180;
  b_ = 45 * M_PI / 180;
  sin45_ = sin(45 * M_PI / 180);

  double ku = 1600;
  kp_tau_ << ku, ku, ku * 0.5;
  ki_tau_ << ku * 0.1, ku * 0.1, ku * 0.01;
  kd_tau_ << ku * 0.04, ku * 0.04, ku * 0.005;
  // PID3 pid_tau(dt, kp_tau_, ki_tau_, kd_tau_);
  pid_tauPtr_.reset(new PID3(dt, kp_tau_, ki_tau_, kd_tau_));
  double ks = 0.00002;
  kp_vel_ << ks, ks, ks * 2;
  ki_vel_ << ks * 0.1, ks * 0.1, ks * 0.1;
  kd_vel_ << 0, 0, 0;
  pid_velPtr_.reset(new PID3(dt, kp_vel_, ki_vel_, kd_vel_));
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
  // get body angle and setpoint in rw axes
  double ref_a = GetXRotatedAboutZ(Q_ref, a_);
  double ref_b = GetXRotatedAboutZ(Q_ref, b_);
  Eigen::Vector3d theta;
  theta(0) = GetXRotatedAboutZ(Q_base, a_);
  theta(1) = GetXRotatedAboutZ(Q_base, b_);
  theta(2) = 2 * asin(Q_base.z());  // z-axis of body quaternion
  return theta;
}

Eigen::Vector3d Rwa::AttitudeSetp(Eigen::Quaterniond Q_ref, double z_ref) {
  Eigen::Vector3d setp;
  setp(0) = GetXRotatedAboutZ(Q_ref, a_);
  setp(1) = GetXRotatedAboutZ(Q_ref, b_);
  setp(2) = z_ref;
  return setp;
}

Eigen::Vector3d Rwa::AttitudeControl(Eigen::Quaterniond Q_ref, Eigen::Quaterniond Q_base, double z_ref) {
  // simple reaction wheel attitude control w/ derivative on measurement pid
  theta_ = AttitudeIn(Q_ref, Q_base);
  setp_ = AttitudeSetp(Q_ref, z_ref);
  setp_ = setp_ - pid_velPtr_->PIDControl(theta_, setp_);
  return pid_tauPtr_->PIDControlWrapped(theta_, setp_);  // Cascaded PID Loop
}

Eigen::Vector3d Rwa::TorqueControl(Eigen::Vector3d tau_ref) {
  // simple reaction wheel torque control (rotated into rw axes)
  Eigen::Vector3d tau;
  tau(0) = (tau_ref(0) - tau_ref(1)) / (2 * sin45_);
  tau(1) = (tau_ref(0) - tau_ref(1)) / (2 * sin45_);
  tau(2) = tau_ref(2);
  return tau;
}