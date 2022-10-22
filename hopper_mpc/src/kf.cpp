#include "hopper_mpc/kf.h"

Kf::Kf(double dt_) {
  dt = dt_;

  A.setIdentity();
  B.setZero();
  C.setIdentity();
  P.setIdentity();

  W.setIdentity();
  W.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * eye3;   // position transition
  W.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * eye3;   // velocity transition
  W.block<3, 3>(9, 9) = PROCESS_NOISE_PFOOT * eye3;  // foot position transition

  V.setIdentity();
  V.block<3, 3>(6, 6) = SENSOR_NOISE_PIMU_REL_FOOT * eye3;  // fk estimation
  V.block<3, 3>(9, 9) = SENSOR_NOISE_VIMU_REL_FOOT * eye3;  // vel estimation

  eye3.setIdentity();
  eye_state.setIdentity();
  // P = P * 3;
}

void Kf::InitState(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d pf, Eigen::Vector3d vf) {
  xhat.setZero();
  xhat.segment<3>(0) = p;
  xhat.segment<3>(3) = v;
  xhat.segment<3>(6) = pf;
  xhat.segment<3>(9) = vf;
  filter_initialized = true;
}

kfVals Kf::EstUpdate(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Quaterniond Q,
                     Eigen::Vector3d a_imu, bool c) {
  // actual measurement
  y.block<3, 1>(0, 0) = p;   // fk estimation
  y.block<3, 1>(3, 0) = v;   // fk estimation
  y.block<3, 1>(6, 0) = pf;  // fk estimation
  y.block<3, 1>(9, 0) = vf;  // vel estimation

  // update A B using latest dt
  A.block<3, 3>(0, 0) = dt * eye3;
  A.block<3, 3>(6, 6) = dt * eye3;

  B(1, 0) = dt;
  B(3, 1) = dt;

  // update W
  W.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
  W.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;
  // update Q R for legs not in contact
  W.block<3, 3>(6, 6) = c * 1e4 * dt * PROCESS_NOISE_PFOOT * eye3;    // foot position transition
  V.block<3, 3>(6, 6) = c * 1e4 * SENSOR_NOISE_PIMU_REL_FOOT * eye3;  // fk estimation
  V.block<3, 3>(9, 9) = c * 1e4 * SENSOR_NOISE_VIMU_REL_FOOT * eye3;  // vel estimation

  // prediction
  xhat = xbar + L * (y - C * xbar);
  P = (eye_state - L * C) * Pbar;
  // process update
  Eigen::Vector3d u = Q.matrix() * a_imu + Eigen::Vector3d(0, 0, -9.81);  // control input u = R*a + a_g
  xbar = A * xhat + B * u;
  Pbar = A * P * A.transpose() + W;
  // recompute Kalman gain
  L = Pbar * C.transpose() * (C * Pbar * C.transpose() + V).inverse();
  // measurement construction
  yhat = C * xbar;
  y_error = y - yhat;

  return kfVals{yhat.segment<3>(0), yhat.segment<3>(3), yhat.segment<3>(6), yhat.segment<3>(9)};
}