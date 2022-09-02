#include "hopper_mpc/mpc_osqp.h"
#include "Eigen/Core"
#include "hopper_mpc/utils.hpp"

Mpc::Mpc(Model model, double dt, int N, Eigen::Matrix3d Jinv, Eigen::Vector3d rh) {
  model_ = model;
  dt_ = dt;
};

//     self.f_max = None  # initialize max forces
//     self.f_min = None  # initialize min forces

//     self.n_x = 12  # number of euler states
//     self.n_u = 6
//     self.A = np.zeros((self.n_x, self.n_x))
//     self.A[0:3, 6:9] = np.eye(3)
//     self.B = np.zeros((self.n_x, self.n_u))
//     self.G = np.zeros(self.n_x)
//     self.G[8] = -self.g
//     self.G_2 = np.zeros((self.n_x, 1))  # required for 2nd order version
//     self.G_2[8] = -self.g
//     self.Ad = np.zeros((self.N, self.n_x, self.n_x))
//     self.Bd = np.zeros((self.N, self.n_x, self.n_u))
//     # self.Gd = np.zeros((self.n_x, 1))
//     self.Gd = self.G * t  # doesn't change, doesn't need updating per timestep
//     self.Q = np.eye(self.n_x)
//     np.fill_diagonal(self.Q, [50., 1., 50., 10., 50., 50., 1., 1., 1., 10., 10., 10.])
//     self.R = np.eye(self.n_u)
//     np.fill_diagonal(self.R, [0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
//     self.x = cp.Variable((N + 1, self.n_x))
//     self.u = cp.Variable((N, self.n_u))

// def mpcontrol(self, x_in, x_ref_in, pf_ref, C, f_max, init):
//     """
//     SQP: since Ak(xk) and Bk(xk) are functions of x, we need to calculate Ak and Bk for each timestep with some x.
//     Since we only have x0 when we run the mpc, we need to guess the rest of the xk over the mpc horizon.
//     An easy way guess x is to use the x generated from the previous MPC run and time shift it.
//     """
//     # print(x_ref_in[0, :] - x_in)
//     self.f_max = f_max  # update f_max
//     N = self.N
//     x_guess = np.zeros((N+1, self.n_x))
//     if init is True:
//         # x_guess = np.tile(x_in, (N, 1))  # For the first MPC run, just repeat x0.
//         x_guess[0, :] = x_in
//         x_guess[1:, :] = x_ref_in  # Use ref traj as initial guess?
//         # calculate better x_guess based on initial x_guess
//         self.gen_dt_dynamics(x_guess, pf_ref)  # bad initial x
//         cost, constr = self.build_qp(x_in, x_ref_in, self.Ad, self.Bd, self.Gd, C)
//         self.solve_qp(cost, constr)
//         x_guess = self.x.value
//     else:
//         x_guess[0, :] = x_in
//         x_guess[1:-1, :] = self.x.value[2:, :]  # time shift.
//         x_guess[-1, :] = self.x.value[-1, :]  # copy last timestep as a dumb approx

//     # x_guess[:, 3:6] = 0
//     # x_guess[:, 9:] = 0
//     # calculate control based on x_guess
//     self.gen_dt_dynamics(x_guess, pf_ref)  # use new x as initial guess
//     cost, constr = self.build_qp(x_in, x_ref_in, self.Ad, self.Bd, self.Gd, C)
//     self.solve_qp(cost, constr)
//     u = self.u.value
//     return u[0, :]

// def gen_dt_dynamics(self, x, pf_ref):  # 1st order version
//     # this formulation keeps f in the body frame
//     # for every MPC horizon timestep, build CT A and B matrices and discretize them
//     dt = self.t
//     rh = self.rh
//     Jinv = self.Jinv
//     n_x = self.n_x  # number of states
//     A = self.A
//     B = self.B
//     for k in range(self.N):
//         rz_phi = rz(x[k, 5])
//         rf = rh + rz_phi @ (pf_ref[k, :] - x[k, 0:3])  # vector from body CoM to footstep location in body frame
//         rhat = hat(rf)  # keeping it in body frame this time
//         J_w_inv = rz_phi @ Jinv @ rz_phi.T  # world frame Jinv
//         A[3:6, 9:] = rz_phi
//         B[6:9, 0:3] = rz_phi.T / self.m
//         B[9:12, 0:3] = J_w_inv @ rz_phi.T @ rhat  # here is where you convert to world frame
//         B[9:12, 3:] = J_w_inv @ rz_phi.T
//         # discretization
//         self.Ad[k, :, :] = np.eye(n_x) + A * dt  # forward euler for comp. speed
//         self.Bd[k, :, :] = B * dt
//     return None

// def gen_dt_dynamics_2nd(self, x, pf_ref):  # 2nd order version
//     # this formulation keeps f in the body frame
//     # for every MPC horizon timestep, build CT A and B matrices and discretize them
//     dt = self.t
//     rh = self.rh
//     Jinv = self.Jinv
//     n_x = self.n_x  # number of states
//     n_u = self.n_u
//     A = self.A
//     B = self.B
//     G = self.G_2
//     for k in range(self.N):
//         rz_phi = rz(x[k, 5])
//         rf = rh + rz_phi @ (pf_ref[k, :] - x[k, 0:3])  # vector from body CoM to footstep location in body frame
//         rhat = hat(rf)  # keeping it in body frame this time
//         J_w_inv = rz_phi @ Jinv @ rz_phi.T  # world frame Jinv
//         A[3:6, 9:] = rz_phi
//         B[6:9, 0:3] = rz_phi.T / self.m
//         B[9:12, 0:3] = J_w_inv @ rz_phi.T @ rhat  # here is where you convert to world frame
//         B[9:12, 3:] = J_w_inv @ rz_phi.T
//         # discretization
//         A_bar = np.hstack((A, B, G))
//         A_bar.resize((n_x + n_u + 1, n_x + n_u + 1))
//         I_bar = np.eye(n_x + n_u + 1)
//         M = I_bar + A_bar * dt + 0.5 * (dt ** 2) * A_bar @ A_bar
//         self.Ad[k, :, :] = M[0:n_x, 0:n_x]
//         self.Bd[k, :, :] = M[0:n_x, n_x:n_x + n_u]
//         self.Gd = M[0:n_x, -1]
//     return None

// def build_qp(self, x_in, x_ref, Ad, Bd, Gd, C):
//     x = self.x
//     u = self.u
//     m = self.m
//     g = self.g
//     mu = self.mu
//     N = self.N
//     n_x = self.n_x
//     n_u = self.n_u
//     Q = self.Q
//     R = self.R
//     u_ref = np.zeros(n_u)
//     u_ref_0 = np.zeros(n_u)
//     cost = 0
//     constr = []
//     for k in range(0, N):
//         Ak = Ad[k, :, :]
//         Bk = Bd[k, :, :]
//         kf = 100 if k == N - 1 else 1  # terminal cost
//         kuf = 0 if k == N - 1 else 1  # terminal cost
//         z = x[k, 2]
//         fx = u[k, 0]
//         fy = u[k, 1]
//         fz = u[k, 2]
//         taux = u[k, 3]
//         tauy = u[k, 4]
//         tauz = u[k, 5]

//         constr += [taux <= 7.78,
//                     taux >= -7.78,
//                     tauy <= 7.78,
//                     tauy >= -7.78,
//                     tauz <= 4,
//                     tauz >= -4]
//         constr += [fy == 0]  # body frame y is always zero
//         # constr += [z >= 0.1]
//         if C[k] == 0:  # even
//             u_ref = u_ref_0[:]  # force ref should be zeros during flight
//             cost += cp.quad_form(x[k + 1, :] - x_ref[k, :], Q * kf) + cp.quad_form(u[k, :] - u_ref, R * kuf)
//             constr += [x[k + 1, :] == Ak @ x[k, :] + Bk @ u[k, :] + Gd,
//                         0 == fx,
//                         0 == fz]
//         else:  # odd
//             u_ref[0] = 40  # 26
//             u_ref[2] = m * g * 2
//             cost += cp.quad_form(x[k + 1, :] - x_ref[k, :], Q * kf) + cp.quad_form(u[k, :] - u_ref, R * kuf)
//             constr += [x[k + 1, :] == Ak @ x[k, :] + Bk @ u[k, :] + Gd,
//                         0 >= fx - mu * fz,
//                         0 >= -fx - mu * fz,
//                         fz >= 0,
//                         fz <= self.f_max[2]]
//                         # z >= 0.1]  #,
//                         # z <= 3]

//     constr += [x[0, :] == x_in]  # initial condition
//     # constr += [x[-1, :] == x_ref[-1, :]]  # final condition
//     return cost, constr

// def solve_qp(self, cost, constr):
//     problem = cp.Problem(cp.Minimize(cost), constr)
//     problem.solve(solver=cp.OSQP)  #, verbose=True)
//     if self.u.value is None or self.x.value is None:
//         raise Exception("\n *** QP FAILED *** \n")
//     return None