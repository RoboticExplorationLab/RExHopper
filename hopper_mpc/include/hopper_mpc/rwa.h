#pragma once
#include "Eigen/Dense"
#include "hopper_mpc/lowpass.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"

class Rwa {                             // The class
 public:                                // Access specifier
  Rwa(std::string bridge, double dt_);  // constructor
  // Eigen::Vector3d q;
  Eigen::Vector3d dq;
  // Eigen::Vector3d q_prev;

  void UpdateState(Eigen::Vector3d dq_in);
  // void UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd);
  Eigen::Vector3d AttitudeCtrl(Eigen::Quaterniond Q_ref, Eigen::Quaterniond Q_base, double z_ref);
  Eigen::Vector3d TorqueCtrl(Eigen::Vector3d tau_ref);
  Eigen::Vector3d theta;
  Eigen::Vector3d setp;

 private:
  double dt;
  double a;
  double b;
  double sin45;

  Eigen::Vector3d kp_tau;
  Eigen::Vector3d ki_tau;
  Eigen::Vector3d kd_tau;
  // PID3 pid_tau;
  Eigen::Vector3d kp_vel;
  Eigen::Vector3d ki_vel;
  Eigen::Vector3d kd_vel;
  // PID3 pid_vel;
  std::unique_ptr<PID3> pid_tauPtr;
  std::unique_ptr<PID3> pid_velPtr;
  // filter
  std::unique_ptr<LowPass3D> lowpassPtr1;
  std::unique_ptr<LowPass3D> lowpassPtr2;

  Eigen::DiagonalMatrix<double, 3> kp_diag;
  Eigen::DiagonalMatrix<double, 3> kd_diag;
  Eigen::Vector2d tau;
  Eigen::Vector3d AttitudeIn(Eigen::Quaterniond Q_base);
  Eigen::Vector3d AttitudeSetp(Eigen::Quaterniond Q_ref, double z_ref);
  double GetXRotatedAboutZ(Eigen::Quaterniond Q_in, double z);
};
