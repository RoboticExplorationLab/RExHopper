#include "hopper_mpc/runner.h"
#include <iostream>
#include "Eigen/Dense"

Runner::Runner(Model model, int N_run, double dt, std::string ctrl, std::string bridge, bool plot, bool fixed, bool spr, bool record) {
  model_ = model;
  N_run_ = N_run;
  dt_ = dt;
  ctrl_ = ctrl;
  plot_ = plot;
  fixed_ = fixed;
  spr_ = spr;
  record_ = record;

  g_ = 9.807;  // should g be defined here?

  u_.setZero(model.n_a);
  L_ = model.leg_dim;
  h0_ = model.h0;
  J_ = model.inertia;
  mu_ = model.mu;

  n_X_ = 13;
  n_U_ = 6;
  X_0_ << 0, 0, h0_, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  X_f_ << 2.5, 0, h0_, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  t_p_ = 0.8;                           // gait period, seconds
  phi_switch_ = 0.5;                    // switching phase, must be between 0 and 1. Percentage of gait spent in contact.
  N_ = 40;                              // mpc prediction horizon length (mpc steps)
  dt_mpc_ = 0.01;                       // mpc sampling time (s), needs to be a factor of N
  N_mpc_ = dt_mpc_ / dt_;               // mpc sampling time (timesteps), repeat mpc every x timesteps
  t_horizon_ = N_ * dt_mpc_;            // mpc horizon time
  N_k_ = N_ * N_mpc_;                   // total mpc prediction horizon length (low-level timesteps)
  t_start_ = 0.5 * t_p_ * phi_switch_;  // start halfway through stance phase
  t_stance_ = t_p_ * phi_switch_;       // time spent in stance
  N_c_ = t_stance_ / dt;                // number of timesteps spent in contact
  std::cout << "before... \n";
  // class definitions
  if (bridge == "hardware") {
    throw "hardware bridge not implemented yet!";
    // TODO: fix this
    // bridgePtr_.reset(new HardwareBridge(model, dt, g_, mu_, fixed, record));
  } else if (bridge == "mujoco") {
    bridgePtr_.reset(new MujocoBridge(model, dt, g_, mu_, fixed, record));
  } else if (bridge == "raisim") {
    bridgePtr_.reset(new RaisimBridge(model, dt, g_, mu_, fixed, record));
  } else {
    throw "Invalid bridge name! Use 'hardware', 'mujoco', or raisim";
  }
  std::cout << "after... \n";
  legPtr_.reset(new Leg(model, dt, g_));

  // initialize variables
  a_in << 0, 0;
  gc_state = "Fall";
  gc_state_prev = gc_state;

};  // constructor

void Runner::Run() {  // Method/function defined inside the class
  bridgePtr_->Init();
  double t = -dt_;
  for (int k = 0; k < N_run_; k++) {
    t += dt_;
    // std::cout << k << "\n";
    u_ << 10000, 10000, 10000, 10000, 10000;
    bridgePtr_->SimRun(u_);  // X, qa, dqa, c, tau, i, v, grf = self.simulator.sim_run(u=self.u)  # run sim
    legPtr_->UpdateState(a_in, Q_base);
    Eigen::Vector3d p_ref;
    p_ref << 0, 0, -0.6;
    Eigen::Vector3d v_ref;
    v_ref << 0, 0, 0;
    u_a = legPtr_->OpSpacePosCtrl(p_ref, v_ref);
    bool s = ContactSchedule(t, 0);
    bool sh = 0;  // for now
    double dz = 0;
    GaitCycleUpdate(s, sh, dz);

    gc_state_prev = gc_state;  // should be last
  }
};

void Runner::GaitCycleUpdate(bool s, bool sh, double dz) {
  if (gc_state == "Cmpr" && dz >= 0) {
    gc_state = "Push";
  } else if (gc_state == "Push" && s == false && sh == false) {
    gc_state = "Rise";
  } else if (gc_state == "Rise" && dz <= 0) {
    gc_state = "Fall";
  } else if (gc_state == "Fall" && sh == true) {
    gc_state = "Cmpr";
  }
};

bool Runner::ContactSchedule(double t, double t0) {
  int phi = int((t - t0) / t_p_) % 1;
  return phi < phi_switch_ ? 1 : 0;
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
  int N_traj = N_run_ - N_sit_;
  int N_ref = N_run_ + N_k_;

  x_ref_0_.resize(12, N_traj);
  for (int i = 0; i < 12; i++) {
    // interpolate positions
    x_ref_0_.row(i) = Eigen::VectorXd::LinSpaced(N_traj, x_0(i), x_f(i));
  }
  // bool C = ContactMap(N_ref, dt_, t_start_, 0);
  Eigen::MatrixXd x_ref_sit;
  int N_sit_ref = N_k_ + N_sit_;  // add MPC horizon
  x_ref_sit.resize(12, N_sit_ref);
  for (int i = 0; i < N_sit_ref; i++) {  // can't use Replicate because it's not dynamic...
    x_ref_sit.block<12, 1>(0, i) = x_f;
  }
  Eigen::MatrixXd x_ref(x_ref_0_.rows() + x_ref_sit.rows(), x_ref_0_.cols());
  x_ref << x_ref_0_, x_ref_sit;
  return x_ref;  // TODO: sine wave for mpc
}
