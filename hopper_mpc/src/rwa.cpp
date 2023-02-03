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
  // dq_ref << 0.0, 0.0, 0.0;  // Make this nonzero to reduce static friction?

  a = 45 * M_PI / 180;
  b = -45 * M_PI / 180;
  sin45 = sin(45 * M_PI / 180);

  double ku;  // use gain of 13 for CoM bisection search.
  double kd;  // 0.1875;
  double kp;
  double ki;
  double kpos;

  if (bridge == "mujoco") {
    ku = 300;  // 800
    kp = 0.6;
    ki = 0.0;  // an integral term would fight the cascaded velocity term
    kd = 0.04;

    kpos = 0.0001;  // 0.0001

  } else {
    ku = 150;  // 180
    kp = 0.6;
    ki = 0.1;   // 0.56;
    kd = 0.02;  // 0.1875;

    kpos = 0.0001;  // 0.0001
  }
  kp_tau << kp, kp, kp * 0.5;
  ki_tau << ki, ki, ki * 0.5;  // ki_tau << 0.1, 0.1, 0.01;
  kd_tau << kd, kd, kd * 0.5;  // 0.04, 0.04, 0.005;
  pid_tauPtr.reset(new PID3(dt, kp_tau * ku, ki_tau * ku, kd_tau * ku));

  double kr = 0.05;  // 0.04
  double krp = 1.0;
  double kri = 0.1;   // 0.1
  double krd = 0.03;  // 0.03
  kp_rs << krp, krp, krp;
  ki_rs << kri, kri, kri;
  kd_rs << krd, krd, krd;
  pid_rsPtr.reset(new PID3(dt, kp_rs * kr, ki_rs * kr, kd_rs * kr));

  double kpp = 1.0;   // flywheel position gain
  double kpi = 0.02;  // gain on integral of position
  double kpd = 0.01;  // roughly equivalent to velocity proportional term
  kp_pos << kpp, kpp, kpp;
  ki_pos << kpi, kpi, kpi;
  kd_pos << kpd, kpd, kpd;
  pid_posPtr.reset(new PID3(dt, kp_pos * kpos, ki_pos * kpos, kd_pos * kpos));
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
  // theta = lowpassPtrRW->Filter(AttitudeIn(Q_base));
  theta = AttitudeIn(Q_base);
  setp = AttitudeSetp(Q_ref, z_ref);

  // setp += pid_velPtr->PIDControl(dq, dq_ref);  // velocity compensation
  setp += pid_posPtr->PIDControl(q, q_ref);  // position compensation

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

Eigen::Vector3d Rwa::RotorVelCtrl() {
  // rotor speed control (mostly just for testing polarity)
  dq_ref << 5.0, 5.0, 5.0;                                  // Make this nonzero to reduce static friction?
  Eigen::Vector3d tau = pid_rsPtr->PIDControl(dq, dq_ref);  // velocity compensation
  // setp = lowpassPtr->Filter(setp);  // filter the vel setpoint
  return tau;
}

Eigen::Vector3d Rwa::RotorPosCtrl() {
  // rotor speed control (mostly just for testing polarity)
  q_ref << 5.0, 5.0, 5.0;
  Eigen::Vector3d tau = pid_rsPtr->PIDControl(q, q_ref);  // velocity compensation
  // setp = lowpassPtr->Filter(setp);  // filter the vel setpoint
  return tau;
}