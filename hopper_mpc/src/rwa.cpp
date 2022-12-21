#include "hopper_mpc/rwa.h"
#include <iostream>
#include "Eigen/Core"

#include "hopper_mpc/utils.hpp"

Rwa::Rwa(std::string bridge, double dt_) {
  dt = dt_;
  q.setZero();
  dq.setZero();
  q_ref.setZero();
  dq_ref.setZero();

  a = 45 * M_PI / 180;
  b = -45 * M_PI / 180;
  sin45 = sin(45 * M_PI / 180);

  double ku;  // use gain of 13 for CoM bisection search.
  double kd;  // 0.1875;
  double kp;
  double ki;

  double ks;
  double ksp;
  double ksi;
  double ksd;

  if (bridge == "mujoco") {
    ku = 800;
    kp = 0.6;
    ki = 0.0;  // an integral term would fight the cascaded velocity term
    kd = 0.04;

    ks = 0.00006;
    ksp = 0.0;
    ksi = 1;  // TODO: run PID on the integral term? the position of the flywheel vels??
    ksd = 0.0;
  } else {
    ku = 60;
    kp = 0.6;
    ki = 0.0;
    kd = 0.04;  // 0.1875;

    // ks = 0.0;
    ks = 0.000006;
    ksp = 0.0;
    ksi = 1;
    ksd = 0.0;
  }

  kp_tau << kp, kp, kp * 0.5;
  ki_tau << ki, ki, ki * 0.5;  // ki_tau << 0.1, 0.1, 0.01;
  kd_tau << kd, kd, kd * 0.5;  // 0.04, 0.04, 0.005;
  pid_tauPtr.reset(new PID3(dt, kp_tau * ku, ki_tau * ku, kd_tau * ku));

  kp_vel << ksp, ksp, ksp * 0.01;
  ki_vel << ksi, ksi, ksi * 0.01;
  kd_vel << ksd, ksd, ksd * 0.01;
  pid_velPtr.reset(new PID3(dt, kp_vel * ks, ki_vel * ks, kd_vel * ks));

  lowpassPtr1.reset(new LowPass3D(dt, 160));
  lowpassPtr2.reset(new LowPass3D(dt, 80));

  double kr = 0.04;
  double krp = 1.0;
  double kri = 0.1;
  double krd = 0.03;
  kp_rs << krp, krp, krp;
  ki_rs << kri, kri, kri;
  kd_rs << krd, krd, krd;
  pid_rsPtr.reset(new PID3(dt, kp_rs * kr, ki_rs * kr, kd_rs * kr));
}

void Rwa::UpdateState(Eigen::Vector3d q_in, Eigen::Vector3d dq_in) {
  // Pull raw actuator joint vel values in from simulator or robot
  q = q_in;
  dq = dq_in;
}

double Rwa::GetXRotatedAboutZ(Eigen::Quaterniond Q_in, double z) {
  // while keeping the original rotation, rotate quaternion frame about z-axis by angle "z"
  // and get rotation about x-axis of the new frame

  Eigen::Quaterniond Q_res = Utils::GenYawQuat(z) * Q_in;
  // double angle = 2 * asin(Q_res.x());
  double angle = Utils::ExtractX(Q_res);
  // std::cout << "Original angle = " << angle << ", New angle = " << angle_new << "\n";
  return angle;
}

Eigen::Vector3d Rwa::AttitudeIn(Eigen::Quaterniond Q_base) {
  // get body angle in rw axes
  Eigen::Quaterniond Q_base_forward = Utils::ExtractYawQuat(Q_base).conjugate() * Q_base;
  theta(0) = GetXRotatedAboutZ(Q_base_forward, a);
  theta(1) = GetXRotatedAboutZ(Q_base_forward, b);
  // theta(2) = 2 * asin(Q_base.z());  // z-axis of body quaternion
  theta(2) = Utils::ExtractZ(Q_base);
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
  // theta = lowpassPtr1->Filter(AttitudeIn(Q_base));
  theta = AttitudeIn(Q_base);
  setp = AttitudeSetp(Q_ref, z_ref);
  // dq_ref << 0.0, 0.0, 0.0;  // Make this nonzero to reduce static friction?
  // setp += lowpassPtr2->Filter(pid_velPtr->PIDControl(dq, dq_ref)); // filter the vel setpoint
  // setp += pid_velPtr->PIDControl(dq, dq_ref);         // velocity compensation
  setp += pid_rposPtr->PIDControl(q, q_ref);          // position compensation!?
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

Eigen::Vector3d Rwa::RotorSpeedCtrl() {
  // rotor speed control (mostly just for testing polarity)
  dq_ref << 5.0, 5.0, 5.0;                                  // Make this nonzero to reduce static friction?
  Eigen::Vector3d tau = pid_rsPtr->PIDControl(dq, dq_ref);  // velocity compensation
  // setp = lowpassPtr->Filter(setp);  // filter the vel setpoint
  return tau;
}