#pragma once
#include "Eigen/Dense"
// #include "hopper_mpc/leg.h"

// state estimator parameters
#define CONTROL_SIZE 6
#define STATE_SIZE 12
#define MEAS_SIZE 12  // TODO: Change these
#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.1
#define PROCESS_NOISE_VFOOT 0.01
#define SENSOR_NOISE_PIMU 0.01
#define SENSOR_NOISE_VIMU 0.01
#define SENSOR_NOISE_PFOOT 0.1
#define SENSOR_NOISE_VFOOT 0.1

struct kfVals {
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d pf;
  Eigen::Vector3d vf;
};

// implement a basic error state KF to estimate robot pose
// assume orientation is known from an IMU
class Kf {
 public:
  Kf(double dt_);
  void InitState(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d pf, Eigen::Vector3d vf);
  kfVals EstUpdate(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d a, Eigen::Vector3d ae);

 private:
  // bool filter_initialized = false;
  // state

  Eigen::Matrix<double, STATE_SIZE, 1> xhat;           // estimation state
  Eigen::Matrix<double, STATE_SIZE, 1> xbar;           // estimation state after process update
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P;     // estimation state covariance
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar;  // estimation state covariance after process update
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A;     // estimation state transition
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> W;     // estimation state transition noise
  Eigen::Matrix<double, STATE_SIZE, CONTROL_SIZE> B;   // control input
  Eigen::Matrix<double, CONTROL_SIZE, 1> u;            // control vector

  // observation
  Eigen::Matrix<double, MEAS_SIZE, 1> y;           //  observation
  Eigen::Matrix<double, MEAS_SIZE, 1> yhat;        // estimated observation
  Eigen::Matrix<double, MEAS_SIZE, 1> y_error;     //  observation error
  Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C;  // estimation state observation
  Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> V;   // estimation state observation noise

  // helper matrices
  Eigen::Matrix<double, 3, 3> eye3;                         // 3x3 identity
  Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> L;           // kalman gain
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> eye_state;  // statexstate identity
  // std::shared_ptr<Leg> legPtr;
  double dt;
};
