#include "hopper_mpc/leg.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/model.h"
#include "hopper_mpc/utils.h"

Leg::Leg(Model model, float dt, float g) {
  q = model.init_q;
  q_prev = q;
  dq = model.init_dq;

  dt_ = dt;
  S_b_ = model.S.block<4, 4>(0, 0);  // actuator selection matrix (just the legs)
  G_ << 0, 0, g;
  a_cal_ << model.init_q(0), model.init_q(2);
  // L_ = model.leg_dim;
  // I_ = model.inertia;
  // m_ = model.m;
  // cl_ = model.cl;
}

void Leg::UpdateState(Eigen::Vector2d a_in, Eigen::Quaterniond Q_base) {
  // Pull raw actuator joint values in from simulator or robot and calibrate encoders
  // Make sure this only happens once per time step
  a_in += a_cal_;
  q(0) = a_in(0);
  q(2) = a_in(1);
  q(1) = q(2) - q(0);
  q(3) = -q(1);
  dq = (q - q_prev) / dt_;
  q_prev = q;
  // dq_prev = dq;
  // Rotate gravity vector to match body orientation
  G_ = Utils::Z(Utils::Q_inv(Q_base), G_)  // world frame to body frame
}

Eigen::Matrix<double, 4, 4> Leg::GenMCG() {}

Eigen::Vector2d Leg::KinInv(Eigen::Vector3d p_ref) {
  // def inv_kinematics(self, xyz):
  //     L0 = self.L[0]
  //     L1 = self.L[1]
  //     L2 = self.L[2]
  //     L3 = self.L[3]
  //     L4 = self.L[4]
  //     L5 = self.L[5]
  //     d = 0  # distance along x b/t motors, 0 for 4-bar link

  //     x = xyz[0]
  // #y = xyz[1]
  //     z = xyz[2]
  //     zeta = np.arctan(L5 / (L3 + L4))
  //     rho = np.sqrt(L5 ** 2 + (L3 + L4) ** 2)
  //     phi = np.arccos((L2 ** 2 + rho ** 2 - (x + d) ** 2 - z ** 2) / (2 * L2 * rho)) - zeta
  //     r1 = np.sqrt((x + d) ** 2 + z ** 2)
  //     ksi = np.arctan2(z, (x + d))
  //     epsilon = np.arccos((r1 ** 2 + L2 ** 2 - rho ** 2) / (2 * r1 * L2))
  //     q2 = ksi - epsilon
  // #print((phi - np.pi - q2) * 180 / np.pi)
  //     xm = L2 * np.cos(q2) + L3 * np.cos(phi - np.pi - q2) - d
  //     zm = L2 * np.sin(q2) - L3 * np.sin(phi - np.pi - q2)
  //     r2 = np.sqrt(xm ** 2 + zm ** 2)
  //     sigma = np.arccos((-L1 ** 2 + r2 ** 2 + L0 ** 2) / (2 * r2 * L0))
  //     q0 = np.arctan2(zm, xm) + sigma
  // #print(np.array([ q0, q2 ]) * 180 / np.pi)
  //     return np.array([q0, q2], dtype=float)
}

Eigen::Vector3d Leg::KinFwd() {}
