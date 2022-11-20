#include "hopper_mpc/rwa.h"
#include <iostream>
#include "Eigen/Core"

#include "hopper_mpc/utils.hpp"

Rwa::Rwa(std::string bridge, double dt_) {
  dt = dt_;
  // q.setZero();  // we don't care about rw actuator pos
  dq.setZero();

  a = 45 * M_PI / 180;
  b = -45 * M_PI / 180;
  sin45 = sin(45 * M_PI / 180);

  double ku;
  double ks;
  if (bridge == "mujoco") {
    ku = 300;
    ks = 0.00001;
  } else {
    ku = 2;
    ks = 0.0000001;
  }
  // use gain of 13 for CoM bisection search.
  // can go as high as 1300 (not sure if necessary)
  double kp = 0.6;
  double ki = 4;       // 0.56;
  double kd = 0.1875;  // 0.1875;
  kp_tau << kp, kp, kp * 0.5;
  ki_tau << ki, ki, ki * 0.5;  // ki_tau << 0.1, 0.1, 0.01;
  kd_tau << kd, kd, kd * 0.5;  // 0.04, 0.04, 0.005;
  pid_tauPtr.reset(new PID3(dt, kp_tau * ku, ki_tau * ku, kd_tau * ku));

  double ksi = 3;
  double ksp = 0.5;
  kp_vel << ks, ks, ks * 0.01;
  ki_vel << ks * ksi, ks * ksi, ks * ksi * 0.01;
  kd_vel << ks * ksp, ks * ksp, ks * ksp * 0.01;
  pid_velPtr.reset(new PID3(dt, kp_vel, ki_vel, kd_vel));

  lowpassPtr1.reset(new LowPass3D(dt, 160));
  lowpassPtr2.reset(new LowPass3D(dt, 160));
}

void Rwa::UpdateState(Eigen::Vector3d dq_in) {
  // Pull raw actuator joint vel values in from simulator or robot
  dq = dq_in;
}

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
  theta(2) = 2 * asin(Q_base.z());  // z-axis of body quaternion (Is it though?)
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
  theta = lowpassPtr1->Filter(AttitudeIn(Q_ref, Q_base));
  setp = AttitudeSetp(Q_ref, z_ref);
  // setp -= pid_velPtr->PIDControlWrapped(theta, setp);
  Eigen::Vector3d setp_dq(0, 0, 0);                                // Make this nonzero to reduce static friction?
  Eigen::Vector3d vel_comp = pid_velPtr->PIDControl(dq, setp_dq);  // velocity compensation
  // setp += lowpassPtr->Filter(vel_comp);  // filter the vel setpoint
  setp += vel_comp;
  setp = lowpassPtr2->Filter(setp);                   // filter the setpoint
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