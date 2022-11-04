#include "hopper_mpc/kf.h"

Kf::Kf(double dt_) {
  dt = dt_;
  eye3.setIdentity();
  eye_state.setIdentity();

  A.setIdentity();
  B.setZero();
  C.setIdentity();
  P.setIdentity();
  Pbar = eye_state * 0.001;  // Initialize Pbar
  W.setIdentity();
  W.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt * eye3;   // base pos transition
  W.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * eye3;   // base vel transition
  W.block<3, 3>(6, 6) = PROCESS_NOISE_PFOOT * dt * eye3;  // foot pos transition
  W.block<3, 3>(9, 9) = PROCESS_NOISE_VFOOT * dt * eye3;  // foot vel transition

  V.setIdentity();
  V.block<3, 3>(0, 0) = SENSOR_NOISE_PIMU * eye3;   // base pos estimation
  V.block<3, 3>(3, 3) = SENSOR_NOISE_VIMU * eye3;   // base vel estimation
  V.block<3, 3>(6, 6) = SENSOR_NOISE_PFOOT * eye3;  // foot pos estimation
  V.block<3, 3>(9, 9) = SENSOR_NOISE_VFOOT * eye3;  // foot vel estimation
}

void Kf::InitState(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d pf, Eigen::Vector3d vf) {
  xhat.setZero();
  xhat.segment<3>(0) = p;
  xhat.segment<3>(3) = v;
  xhat.segment<3>(6) = pf;
  xhat.segment<3>(9) = vf;
  // filter_initialized = true;
}

kfVals Kf::EstUpdate(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d a, Eigen::Vector3d ae) {
  // actual measurement
  y.segment<3>(0) = p;   // fk estimation
  y.segment<3>(3) = v;   // fk estimation
  y.segment<3>(6) = pf;  // fk estimation
  y.segment<3>(9) = vf;  // vel estimation

  // update A B using latest dt
  A.block<3, 3>(0, 3) = dt * eye3;  // TODO: Varying dt
  A.block<3, 3>(6, 9) = dt * eye3;

  B.block<3, 3>(3, 0) = dt * eye3;
  B.block<3, 3>(9, 3) = dt * eye3;

  // update W
  W.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt * eye3;
  W.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * eye3;
  W.block<3, 3>(6, 6) = PROCESS_NOISE_PFOOT * dt * eye3;  // foot position transition
  W.block<3, 3>(9, 9) = PROCESS_NOISE_VFOOT * dt * eye3;  // foot position transition

  // process update
  u.segment<3>(0) = a;   // + Eigen::Vector3d(0, 0, -9.81);  // control input u = R*a + a_g
  u.segment<3>(3) = ae;  // + Eigen::Vector3d(0, 0, -9.81);  // TODO: check whether grav is needed with this imu
  xbar = A * xhat + B * u;
  Pbar = A * P * A.transpose() + W;

  // measurement update
  xhat = xbar + L * (y - C * xbar);
  P = (eye_state - L * C) * Pbar;

  // recompute Kalman gain
  L = Pbar * C.transpose() * (C * Pbar * C.transpose() + V).inverse();

  // measurement construction
  yhat = C * xbar;
  // y_error = y - yhat;

  return kfVals{yhat.segment<3>(0), yhat.segment<3>(3), yhat.segment<3>(6), yhat.segment<3>(9)};
}