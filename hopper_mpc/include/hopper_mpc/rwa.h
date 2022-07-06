#pragma once
#include "Eigen/Dense"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"

class Rwa {       // The class
 public:          // Access specifier
  Rwa(float dt);  // constructor
  // Eigen::Vector3d q;
  // Eigen::Vector3d dq;
  // Eigen::Vector3d q_prev;

  // void UpdateState(Eigen::Vector3d q_in);
  // void UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd);
  Eigen::Vector3d AttitudeControl(Eigen::Quaterniond Q_ref, Eigen::Quaterniond Q_base, double z_ref);
  Eigen::Vector3d TorqueControl(Eigen::Vector3d tau_ref);
  Eigen::Vector3d theta_;
  Eigen::Vector3d setp_;

 private:
  float dt_;
  double a_;
  double b_;
  double sin45_;

  Eigen::Vector3d kp_tau_;
  Eigen::Vector3d ki_tau_;
  Eigen::Vector3d kd_tau_;
  PID3 pid_tau;
  Eigen::Vector3d kp_vel_;
  Eigen::Vector3d ki_vel_;
  Eigen::Vector3d kd_vel_;
  PID3 pid_vel;

  Eigen::DiagonalMatrix<double, 3> kp_diag_;
  Eigen::DiagonalMatrix<double, 3> kd_diag_;
  Eigen::Vector2d tau_;
  Eigen::Vector3d AttitudeIn(Eigen::Quaterniond Q_ref, Eigen::Quaterniond Q_base);
  Eigen::Vector3d AttitudeSetp(Eigen::Quaterniond Q_ref, double z_ref);
  double GetXRotatedAboutZ(Eigen::Quaterniond Q_in, double z);
};
