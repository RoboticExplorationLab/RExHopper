#pragma once
#include "Eigen/Dense"
// #include "hopper_mpc/leg.h"

// state estimator parameters
#define STATE_SIZE 9
#define MEAS_SIZE 28  // TODO: Change this
#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.01
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1
#define SENSOR_NOISE_ZFOOT 0.001

// implement a basic error state KF to estimate robot pose
// assume orientation is known from an IMU (state.root_rot_mat)
class Ekf {
 public:
  Ekf();
  // Ekf(bool assume_flat_ground_);
  void InitState(Eigen::Vector3d p, Eigen::Quaterniond Q, Eigen::Vector3d v, Eigen::Vector3d w);
  void EstUpdate(Eigen::Vector3d p, Eigen::Quaterniond Q, Eigen::Vector3d v, Eigen::Vector3d w, double dt);

 private:
  bool filter_initialized = false;
  // state
  Eigen::Matrix<double, STATE_SIZE, 1> x;              // estimation state
  Eigen::Matrix<double, STATE_SIZE, 1> xbar;           // estimation state after process update
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P;     // estimation state covariance
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar;  // estimation state covariance after process update
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A;     // estimation state transition
  Eigen::Matrix<double, STATE_SIZE, 3> B;              // estimation state transition
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q;     // estimation state transition noise

  // observation
  Eigen::Matrix<double, MEAS_SIZE, 1> y;            //  observation
  Eigen::Matrix<double, MEAS_SIZE, 1> yhat;         // estimated observation
  Eigen::Matrix<double, MEAS_SIZE, 1> error_y;      // estimated observation
  Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y;     // S^-1*error_y
  Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C;   // estimation state observation
  Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC;  // S^-1*C
  Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R;    // estimation state observation noise

  // helper matrices
  Eigen::Matrix<double, 3, 3> eye3;                // 3x3 identity
  Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S;   // Innovation (or pre-fit residual) covariance
  Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K;  // kalman gain

  // bool assume_flat_ground = false;

  // variables to process foot force
  // double f_smooth;  // smooth_foot_force
  // double c_est;
};
