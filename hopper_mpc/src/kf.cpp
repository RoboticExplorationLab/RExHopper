#include "hopper_mpc/kf.h"

Ekf::Ekf() {
  // constructor
  eye3.setIdentity();
  // C is fixed
  C.setZero();
  C.block<3, 3>(3, 0) = -eye3;     //-pos
  C.block<3, 3>(3, 6 + 3) = eye3;  // foot pos
  C.block<3, 3>(3 + 3, 3) = eye3;  // vel
  C(6, 6 + 3 + 2) = 1;             // height z of foot

  // Q R are fixed
  Q.setIdentity();
  Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * eye3;           // position transition
  Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * eye3;           // velocity transition
  Q.block<3, 3>(6 + 3, 6 + 3) = PROCESS_NOISE_PFOOT * eye3;  // foot position transition

  R.setIdentity();
  R.block<3, 3>(3, 3) = SENSOR_NOISE_PIMU_REL_FOOT * eye3;          // fk estimation
  R.block<3, 3>(3 + 3, 3 + 3) = SENSOR_NOISE_VIMU_REL_FOOT * eye3;  // vel estimation
  R(6, 6) = SENSOR_NOISE_ZFOOT;                                     // height z estimation

  // set A to identity
  A.setIdentity();

  // set B to zero
  B.setZero();
}

void Ekf::InitState(Eigen::Quaterniond Q_base, Eigen::Vector3d pfb) {
  filter_initialized = true;
  P.setIdentity();
  P = P * 3;

  // set initial value of x
  x.setZero();
  x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.09);  // TODO: Should always match starting p
  x.segment<3>(9) = Q_base.matrix() * pfb + x.segment<3>(0);
}

void Ekf::EstUpdate(Eigen::Quaterniond Q_base, Eigen::Vector3d pfb, Eigen::Vector3d vfb, Eigen::Vector3d a_imu, double dt, bool c) {
  // update A B using latest dt
  A.block<3, 3>(0, 3) = dt * eye3;
  B.block<3, 3>(3, 0) = dt * eye3;

  // control input u is Ra + ag
  Eigen::Vector3d u = Q_base.matrix() * a_imu + Eigen::Vector3d(0, 0, -9.81);

  // // contact estimation, do something very simple first
  // if (state.movement_mode == 0) {  // stand
  //   for (int i = 0; i < NUM_LEG; ++i) estimated_contacts[i] = 1.0;
  // } else {  // walk
  //   for (int i = 0; i < NUM_LEG; ++i) {
  //     // estimated_contacts[i] = std::min(std::max((state.foot_force(i)) / (70.0 - 0.0), 0.0), 1.0);
  //     estimated_contacts[i] = 1.0 / (1.0 + std::exp(-(state.foot_force(i) - 60.0)));
  //   }
  // }

  // update Q
  Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
  Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;
  // update Q R for legs not in contact
  Q.block<3, 3>(9, 9) = c * 1e4 * dt * PROCESS_NOISE_PFOOT * eye3;    // foot position transition
  R.block<3, 3>(3, 3) = c * 1e4 * SENSOR_NOISE_PIMU_REL_FOOT * eye3;  // fk estimation
  R.block<3, 3>(6, 6) = c * 1e4 * SENSOR_NOISE_VIMU_REL_FOOT * eye3;  // vel estimation

  // process update
  xbar = A * x + B * u;
  Pbar = A * P * A.transpose() + Q;

  // measurement construction
  yhat = C * xbar;

  // actual measurement
  y.block<3, 1>(3, 0) = Q_base.matrix() * pfb;  // fk estimation
  Eigen::Vector3d vf = -vfb - Utils::skew(a_imu) * pfb;
  y.block<3, 1>(3 + 3, 0) = (1.0 - c) * x.segment<3>(3) + c * state.root_rot_mat * vf;  // vel estimation

  y(6) = (1.0 - c) * (x(2) + fk_pos(2));  // height z estimation

  S = C * Pbar * C.transpose() + R;
  S = 0.5 * (S + S.transpose());

  error_y = y - yhat;
  Serror_y = S.fullPivHouseholderQr().solve(error_y);

  x = xbar + Pbar * C.transpose() * Serror_y;

  SC = S.fullPivHouseholderQr().solve(C);
  P = Pbar - Pbar * C.transpose() * SC * Pbar;
  P = 0.5 * (P + P.transpose());

  // reduce position drift
  if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
    P.block<2, 16>(0, 2).setZero();
    P.block<16, 2>(2, 0).setZero();
    P.block<2, 2>(0, 0) /= 10.0;
  }

  // // final step
  // // put estimated values back to A1CtrlStates& state
  // if (c_est < 0.5) {
  //   state.c_est = false;
  // } else {
  //   state.c_est = true;
  // }

  //    std::cout << x.transpose() <<std::endl;
  state.estimated_root_pos = x.segment<3>(0);
  state.estimated_root_vel = x.segment<3>(3);

  // state.root_pos = x.segment<3>(0);
  // state.root_lin_vel = x.segment<3>(3);
}