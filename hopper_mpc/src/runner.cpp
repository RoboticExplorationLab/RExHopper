#include "hopper_mpc/runner.h"
#include <iostream>
#include "Eigen/Dense"
#include "hopper_mpc/plots.hpp"

Runner::Runner(Model model_, int N_run_, double dt_, std::string ctrl_, std::string bridge_, bool plot_, bool fixed_, bool spr_,
               bool record_) {
  model = model_;
  N_run = N_run_;
  dt = dt_;
  ctrl = ctrl_;
  plot = plot_;
  fixed = fixed_;
  spr = spr_;
  record = record_;

  g = model.g;  // should g be defined here?
  L = model.leg_dim;
  h0 = 0.5;  // should match the number used in the mjcf!!
  J = model.inertia;
  mu = model.mu;

  n_X = 13;
  n_U = 6;

  t_p = 0.8;                         // gait period, seconds
  phi_switch = 0.5;                  // switching phase, must be between 0 and 1. Percentage of gait spent in contact.
  N = 40;                            // mpc prediction horizon length (mpc steps)
  dt_mpc = 0.01;                     // mpc sampling time (s), needs to be a factor of N
  N_mpc = dt_mpc / dt;               // mpc sampling time (timesteps), repeat mpc every x timesteps
  t_horizon = N * dt_mpc;            // mpc horizon time
  N_k = N * N_mpc;                   // total mpc prediction horizon length (low-level timesteps)
  t_start = 0.5 * t_p * phi_switch;  // start halfway through stance phase
  t_stance = t_p * phi_switch;       // time spent in stance
  N_c = t_stance / dt;               // number of timesteps spent in contact

  // class definitions
  if (bridge_ == "hardware") {
    // bridgePtr_.reset(new HardwareBridge(model, dt, g_, mu_, fixed, record));
  } else if (bridge_ == "mujoco") {
    bridgePtr.reset(new MujocoBridge(model, dt, fixed, record));
  } else if (bridge_ == "raisim") {
    bridgePtr.reset(new RaisimBridge(model, dt, fixed, record));
  } else {
    throw "Invalid bridge name! Use 'hardware', 'mujoco', or 'raisim'";
  }
  // initialize state
  p << 0, 0, model.h0;  // must match starting position in mjcf!!
  Q.coeffs() << 0, 0, 0, 1;
  v.setZero();
  w.setZero();
  // initialize reference state
  p_ref << 2, 0, model.h0;
  Q_ref.coeffs() << 0, 0, 0, 1;
  v_ref.setZero();
  w_ref.setZero();

  // initialize gait cycle state
  gc_state = "Cmpr";
  gc_state_prev = gc_state;
  gc_id = 3;

  ctrlMode = "Force";                                                // "Torque&Pos"
  qla_ref = (model.S.transpose() * model.q_init).block<2, 1>(0, 0);  // convert from full joint space to actuated joint space
  qla_ref << qla_ref(0), qla_ref(1) + 0.5;
  peb_ref << 0, 0, -model.h0 * 2;  // desired operational space leg position in body frame
  veb_ref.setZero();               // desired operational space leg velocity
  fb_ref.setZero();                // desired operational space leg force
  u.setZero();                     // ctrl Torques
  legPtr = std::make_shared<Leg>(model, dt);
  rwaPtr = std::make_shared<Rwa>(dt);

  gaitPtr.reset(new Gait(model, dt, peb_ref, &legPtr, &rwaPtr));  // gait controller class

  // variables for CircleTest
  x1 = 0;
  z1 = -0.4;
  z = -0.3;  // peb_ref(2);
  r = 0.1;
  flip = -1;
};

void Runner::Run() {  // Method/function defined inside the class
  bridgePtr->Init();
  double t = -dt;

  std::vector<double> theta_x(N_run), theta_y(N_run), theta_z(N_run), setp_x(N_run), setp_y(N_run), setp_z(N_run), q0(N_run), q2(N_run),
      q0_ref(N_run), q2_ref(N_run), peb_x(N_run), peb_z(N_run), peb_refx(N_run), peb_refz(N_run), tau_0(N_run), tau_1(N_run), tau_2(N_run),
      tau_3(N_run), tau_4(N_run), tau_ref0(N_run), tau_ref1(N_run), tau_ref2(N_run), tau_ref3(N_run), tau_ref4(N_run), dq_0(N_run),
      dq_1(N_run), dq_2(N_run), dq_3(N_run), dq_4(N_run), dq_ref0(N_run), dq_ref1(N_run), dq_ref2(N_run), dq_ref3(N_run), dq_ref4(N_run),
      p_z(N_run), p_refz(N_run), sh_hist(N_run), s_hist(N_run), gc_state_hist(N_run);

  for (int k = 0; k < N_run; k++) {
    t += dt;
    auto [p, Q, v, w, qa, dqa, sh] = bridgePtr->SimRun(u, qla_ref, ctrlMode);  // still need c, tau, i, v, grf
    legPtr->UpdateState(qa.block<2, 1>(0, 0), Q);                              // grab first two actuator pos values
    bool s = ContactSchedule(t, 0);
    GaitCycleUpdate(s, sh, v(2));               // TODO: should use v_g instead?
    Eigen::Vector3d peb = legPtr->KinFwd();     // Pos of End-effector in Body frame (P.E.B.)
    Eigen::Vector3d pe = p + Q.matrix() * peb;  // position of the foot in world frame

    auto [u_, qla_ref_, ctrlMode_] = gaitPtr->uRaibert(gc_state, gc_state_prev, p, Q, v, w, p_ref, Q_ref, v_ref, w_ref);
    u = u_;
    qla_ref = qla_ref_;
    ctrlMode = ctrlMode_;

    gc_state_prev = gc_state;  // should be last // u << 0.1, 0.1, 0.1, 0.1, 0.1;
    // std::cout << k << "\n";
    // std::cout << "u = " << u(0) << ", " << u(1) << ", " << u(2) << ", " << u(3) << ", "
    //           << ", " << u(4) << "\n";
    // std::cout << "body frame foot pos = " << peb(0) << ", " << peb(1) << ", " << peb(2) << "\n";
    if (plot == true) {
      theta_x.at(k) = rwaPtr->theta(0);
      theta_y.at(k) = rwaPtr->theta(1);
      theta_z.at(k) = rwaPtr->theta(2);
      setp_x.at(k) = rwaPtr->setp(0);
      setp_y.at(k) = rwaPtr->setp(1);
      setp_z.at(k) = rwaPtr->setp(2);
      q0.at(k) = qa(0) * 180 / M_PI;
      q2.at(k) = qa(1) * 180 / M_PI;
      q0_ref.at(k) = qla_ref(0) * 180 / M_PI;
      q2_ref.at(k) = qla_ref(1) * 180 / M_PI;
      peb_x.at(k) = peb(0);
      peb_z.at(k) = peb(2);
      peb_refx.at(k) = gaitPtr->peb_ref(0);
      peb_refz.at(k) = gaitPtr->peb_ref(2);
      tau_0.at(k) = bridgePtr->tau(0);
      tau_1.at(k) = bridgePtr->tau(1);
      tau_2.at(k) = bridgePtr->tau(2);
      tau_3.at(k) = bridgePtr->tau(3);
      tau_4.at(k) = bridgePtr->tau(4);
      tau_ref0.at(k) = bridgePtr->tau_ref(0) * 7;
      tau_ref1.at(k) = bridgePtr->tau_ref(1) * 7;
      tau_ref2.at(k) = bridgePtr->tau_ref(2);
      tau_ref3.at(k) = bridgePtr->tau_ref(3);
      tau_ref4.at(k) = bridgePtr->tau_ref(4);

      dq_0.at(k) = dqa(0);
      dq_1.at(k) = dqa(1);
      dq_2.at(k) = dqa(2);
      dq_3.at(k) = dqa(3);
      dq_4.at(k) = dqa(4);

      p_z.at(k) = p(2);
      p_refz.at(k) = p_ref(2);
      sh_hist.at(k) = sh;
      s_hist.at(k) = s;

      gc_state_hist.at(k) = gc_id;
    }
  }
  bridgePtr->End();
  if (plot == true) {
    Plots::OpSpacePos(N_run, peb_x, peb_z, peb_refx, peb_refz);
    Plots::Plot2(N_run, "Joint Angular Positions", "q0", q0, q0_ref, "q2", q2, q2_ref, 0);
    Plots::Plot3(N_run, "Contact Timing", "Body Z Pos", p_z, p_refz, "Contact", sh_hist, s_hist, "Gait Cycle State", gc_state_hist,
                 gc_state_hist, 0);
    Plots::Plot3(N_run, "Theta vs Timesteps", "Theta_x", theta_x, setp_x, "Theta_y", theta_y, setp_y, "Theta_z", theta_z, setp_z, 0);
    Plots::Plot5(N_run, "Tau vs Timesteps", "Tau_0", tau_0, tau_ref0, "Tau_1", tau_1, tau_ref1, "Tau_2", tau_2, tau_ref2, "Tau_3", tau_3,
                 tau_ref3, "Tau_4", tau_4, tau_ref4, 60);
    Plots::Plot5(N_run, "dq vs Timesteps", "dq_0", dq_0, dq_0, "dq_1", dq_1, dq_1, "dq_2", dq_2, dq_2, "dq_3", dq_3, dq_3, "dq_4", dq_4,
                 dq_4, 0);
  }
};

void Runner::GaitCycleUpdate(bool s, bool sh, double dz) {
  if (gc_state == "Cmpr" && dz >= 0) {
    gc_state = "Push";
    gc_id = 0;
    // } else if (gc_state == "Push" && s == false && sh == false) {
  } else if (gc_state == "Push" && sh == false) {
    gc_state = "Rise";
    gc_id = 1;
  } else if (gc_state == "Rise" && dz <= 0) {
    gc_state = "Fall";
    gc_id = 2;
  } else if (gc_state == "Fall" && sh == true) {
    gc_state = "Cmpr";
    gc_id = 3;
  }
};

bool Runner::ContactSchedule(double t, double t0) {
  int phi = int((t - t0) / t_p) % 1;
  return phi < phi_switch ? 1 : 0;
}

bool Runner::ContactMap(int N, double dt, double ts, double t0) {
  // generate vector of scheduled contact states over the mpc's prediction horizon
  bool C[N] = {false};
  for (int k = 0; k < N; k++) {
    C[k] = ContactSchedule(ts, t0);
    ts += dt;
  }
  return C;
}

Eigen::MatrixXd Runner::RefTraj(Eigen::Matrix<double, 12, 1> x_0, Eigen::Matrix<double, 12, 1> x_f) {
  int N_traj = N_run - N_sit;
  int N_ref = N_run + N_k;

  x_ref_0.resize(12, N_traj);
  for (int i = 0; i < 12; i++) {
    // interpolate positions
    x_ref_0.row(i) = Eigen::VectorXd::LinSpaced(N_traj, x_0(i), x_f(i));
  }
  // bool C = ContactMap(N_ref, dt_, t_start_, 0);
  Eigen::MatrixXd x_ref_sit;
  int N_sit_ref = N_k + N_sit;  // add MPC horizon
  x_ref_sit.resize(12, N_sit_ref);
  for (int i = 0; i < N_sit_ref; i++) {  // can't use Replicate because it's not dynamic...
    x_ref_sit.block<12, 1>(0, i) = x_f;
  }
  Eigen::MatrixXd x_ref(x_ref_0.rows() + x_ref_sit.rows(), x_ref_0.cols());
  x_ref << x_ref_0, x_ref_sit;
  return x_ref;  // TODO: sine wave for mpc
}

void Runner::CircleTest() {
  // edits peb_ref in-place
  z += 0.001 * flip;  // std::cout << z << "\n";
  if (z <= -0.5 || z >= -0.3) {
    flip *= -1;
  }
  double x = (sqrt(pow(r, 2) - pow(z - z1, 2)) + x1) * -flip;
  if (isnan(x)) {
    x = 0;
  }
  peb_ref(0) = x;
  peb_ref(2) = z;

  // std::cout << "peb_ref = " << peb_ref(0) << ", " << peb_ref(1) << ", " << peb_ref(2) << "\n";
}