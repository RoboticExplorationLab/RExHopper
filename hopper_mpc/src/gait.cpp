#include "hopper_mpc/gait.h"

Gait::Gait(Model model, double dt, Eigen::Matrix<double, 13, 1> X_f) {
  model = model;
  Eigen::Vector3d kp;
  Eigen::Vector3d ki;
  Eigen::Vector3d kd;
  kp << 0.02, 0.08, 0;
  ki << 0.2, 0.2, 0;
  kd << 0.01, 0.02, 0;
  PID3 pid_pdot(dt, kp, ki, kd);
  z_ref_ = 0;
}

Gait::U_Raibert(std::string state, std::string state_prev, Eigen::Matrix<double, 13, 1> X_in, Eigen::Matrix<double, 12, 1> x_ref) {}
// def u_raibert(self, state, state_prev, X_in, x_ref):
// #continuous raibert hopping
// p = X_in[0:3]
// Q_base = X_in[3:7]
// pdot = X_in[7:10]
// p_ref = x_ref[100, 0:3]  # raibert hopping only looks at position ref
// z = 2 * np.arcsin(Q_base[3])  # z-axis of body quaternion
// #z = utils.quat2euler(Q_base)[2]
// Q_z = np.array([np.cos(z / 2), 0, 0, np.sin(z / 2)]).T
// Q_z_inv = utils.Q_inv(Q_z)
// pdot_ref = -self.pid_pdot.pid_control(inp=utils.Z(Q_z_inv, p), setp=utils.Z(Q_z_inv, p_ref))  # adjust for yaw
// pdot_ref = utils.Z(Q_z, pdot_ref)  # world frame -> body frame
// if np.linalg.norm(self.X_f[0:2] - X_in[0:2]) >= 1:
//     v_ref = p_ref - p
//     self.z_ref = np.arctan2(v_ref[1], v_ref[0])  # desired yaw
// k_b = (np.clip(np.linalg.norm(self.X_f[0:2] - X_in[0:2]), 0.5, 1) + 2)/3  # "Braking" gain based on dist
// h = self.h * k_b
// kr = .15 / k_b  # .15 / k_b "speed cancellation" constant
// kt = 0.4  # gain representing leap period accounting for vertical jump velocity at toe-off
// if state == 'Return':
//     if state_prev == 'Leap':  # find new footstep position based on desired speed and current speed
//         self.x_des = raibert_x(kr, kt, pdot, pdot_ref) + p  # world frame desired footstep position
//         self.x_des[2] = 0  # enforce footstep location is on ground plane
//     if pdot[2] >= 0:  # recognize that robot is still rising
//         self.target[2] = -h  # pull leg up to prevent stubbing
//     else:
//         self.target[2] = -h * 5.5 / 3  # brace for impact
// elif state == 'HeelStrike':
//     self.target[2] = -h * 4.5 / 3
// elif state == 'Leap':
//     self.target[2] = -h * 5.5 / 3
// else:
//     raise NameError('INVALID STATE')

// Q_ref = utils.Q_inv(utils.vec_to_quat(self.x_des - p))
// self.u[0:2] = self.controller.wb_pos_control(target=self.target)
// self.u[2:], thetar, setp = self.moment.rw_control(Q_ref, Q_base, self.z_ref)
// return self.u, thetar, setp