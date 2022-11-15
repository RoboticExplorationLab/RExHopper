#include "hopper_mpc/leg.h"
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "codegen/DEL.h"
#include "codegen/Jac.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/utils.hpp"

Leg::Leg(Model model_, double dt_) {
  model = model_;
  q = model.q_init.block<4, 1>(0, 0);
  q_prev = q;
  dq = model.dq_init.block<4, 1>(0, 0);

  dt = dt_;

  S_b = model.S.block<4, 4>(0, 0);  // actuator selection matrix (just the legs)
  gb << 0, 0, model.g;              // initialize body frame gravity vector
  gb_init << 0, 0, model.g;
  L0 = model.leg_dim(0);
  L1 = model.leg_dim(1);
  L2 = model.leg_dim(2);
  L3 = model.leg_dim(3);
  L4 = model.leg_dim(4);
  L5 = model.leg_dim(5);

  s_pol = model.s_pol;
  K << model.K * 7 / 5, model.K * 7 / 5, model.K;
  UpdateGains(K, K * 0.02);
};

void Leg::UpdateState(Eigen::Vector2d qa_in, Eigen::Quaterniond Q_base) {
  // Update joint info
  q(0) = qa_in(0);
  q(2) = qa_in(1);
  q(1) = q(2) - q(0);
  q(3) = -q(1);
  dq = (q - q_prev) / dt;
  q_prev = q;
  // dq_prev = dq;
  // Rotate gravity vector to match body orientation
  gb = Q_base.inverse().matrix() * gb_init;  // world frame to body frame
  GenMCG();
  GenJac();
  GenMx();
  qa << q(0), q(2);  // same thing as qa = q_in, but okay
  dqa << dq(0), dq(2);
};

void Leg::UpdateGains(Eigen::Vector3d kp, Eigen::Vector3d kd) {
  // Use this to update wbc PD gains in real time
  kp_diag = kp.asDiagonal();
  kd_diag = kd.asDiagonal();
}

Eigen::Matrix<double, 2, 1> Leg::KinInv(Eigen::Vector3d p_ref) {
  double x = p_ref(0);
  double z = p_ref(2);
  double zeta = atan(L5 / (L3 + L4));
  double rho = sqrt(pow(L5, 2) + pow((L3 + L4), 2));  // np.sqrt(L5 ** 2 + (L3 + L4) ** 2)
  double phi = acos((pow(L2, 2) + pow(rho, 2) - pow(x, 2) - pow(z, 2)) / (2 * L2 * rho)) - zeta;
  double r1 = sqrt(pow(x, 2) + pow(z, 2));
  double ksi = atan2(z, x);
  double epsilon = acos((pow(r1, 2) + pow(L2, 2) - pow(rho, 2)) / (2 * r1 * L2));
  double q2 = ksi - epsilon;
  double xm = L2 * cos(q2) + L3 * cos(phi - M_PI - q2);
  double zm = L2 * sin(q2) - L3 * sin(phi - M_PI - q2);
  double r2 = sqrt(pow(xm, 2) + pow(zm, 2));
  double sigma = acos((-pow(L1, 2) + pow(r2, 2) + pow(L0, 2)) / (2 * r2 * L0));
  double q0 = atan2(zm, xm) + sigma;
  qa_out << q0, q2;
  return qa_out;
};

Eigen::Vector3d Leg::KinFwd() {
  double q0 = q(0);
  double q2 = q(2);
  double x0a = L0 * cos(q0);
  double z0a = L0 * sin(q0);
  double rho = sqrt(pow(x0a, 2) + pow(z0a, 2));  // rho = sp.sqrt((x0a + d) ** 2 + z0a ** 2)
  double x1a = L2 * cos(q2);
  double z1a = L2 * sin(q2);
  double h = sqrt(pow(x0a - x1a, 2) + pow(z0a - z1a, 2));                    // h = sp.sqrt((x0a - x1a) ** 2 + (z0a - z1a) ** 2)
  double mu = acos((pow(L3, 2) + pow(h, 2) - pow(L1, 2)) / (2 * L3 * h));    // mu = sp.acos((l3 ** 2 + h ** 2 - l1 ** 2) / (2 * l3 * h))
  double eta = acos((pow(h, 2) + pow(L2, 2) - pow(rho, 2)) / (2 * h * L2));  // eta = sp.acos((h ** 2 + l2 ** 2 - rho ** 2) / (2 * h * l2))
  double alpha = M_PI - (eta + mu) + q2;
  double xa = L2 * cos(q2) + (L3 + L4) * cos(alpha) + L5 * cos(alpha - M_PI / 2);
  double ya = 0;
  double za = L2 * sin(q2) + (L3 + L4) * sin(alpha) + L5 * cos(alpha - M_PI / 2);
  Eigen::Vector3d fwd_kin;
  fwd_kin << xa, ya, za;
  return fwd_kin;
};

Eigen::Vector3d Leg::GetVel() {
  return Ja * dqa;
}

void Leg::GenMCG() {
  sym::Del<double>(q, dq, gb, model.m, model.leg_dim, model.l_c0, model.l_c1, model.l_c2, model.l_c3, model.I, &M, &C, &G);
};

void Leg::GenJac() {
  sym::Jac<double>(q, dq, gb, model.leg_dim, &Ja);  // TODO: Fix this
};

void Leg::GenMx() {
  J.setZero();
  J.block<3, 1>(0, 0) = Ja.block<3, 1>(0, 0);
  J.block<3, 1>(0, 2) = Ja.block<3, 1>(0, 1);
  Mx_inv = J * (M.inverse() * J.transpose());  // TODO: Check that this is symmetric
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd(Mx_inv, Eigen::ComputeFullU);
  Eigen::Matrix<double, 3, 3> U = svd.matrixU();
  Eigen::Vector3d s = svd.singularValues();
  for (int i = 0; i < 3; i++) {
    // cut off singular values which cause control problems
    if (s(i) < singularity_thresh) {
      s(i) = 0;
    } else {
      s(i) = 1 / s(i);
    }
  }
  Mx = U * (s.asDiagonal() * U.transpose());  // V = U.transpose(), so V.transpose() = U
}

Eigen::Vector2d Leg::OpSpacePosCtrl(Eigen::Vector3d p_ref, Eigen::Vector3d v_ref) {
  Eigen::Vector3d p = KinFwd();
  Eigen::Vector3d v = GetVel();
  Eigen::Vector3d pdd_ref;
  // pdd_ref = (p_ref - p) + (v_ref - v);
  pdd_ref = kp_diag * (p_ref - p) + kd_diag * (v_ref - v);
  Eigen::Vector3d f;
  f = Mx * pdd_ref;
  tau = Ja.transpose() * f;  // u = tau.flatten() - spring.fn_spring(leg.q[0], leg.q[2])
  return -tau;
}

Eigen::Vector2d Leg::OpSpaceForceCtrl(Eigen::Vector3d f) {
  Eigen::Matrix<double, 3, 2> Ja;
  tau = Ja.transpose() * f;  // u = (Ja.T @ force).flatten() - spring.fn_spring(leg.q[0], leg.q[2])
  return -tau;
}

Eigen::Vector2d Leg::KinInvPosCtrl(Eigen::Vector3d pe_ref, double kp, double kd) {
  // double kp = model.k_kin(0);
  // double kd = model.k_kin(1);

  tau = kp * (qa - KinInv(pe_ref)) + kd * dqa;

  return tau;
}