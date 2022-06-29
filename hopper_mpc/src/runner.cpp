#include "hopper_mpc/runner.h"
#include <iostream>
#include "Eigen/Dense"
#include "hopper_mpc/bridge_mujoco.h"

Runner::Runner(Model model, int N_run, double dt, std::string ctrl, bool plot, bool fixed, bool spr, bool record, bool recalc) {
  N_run_ = N_run;
  dt_ = dt;
  ctrl_ = ctrl;
  plot_ = plot;
  fixed_ = fixed;
  spr_ = spr;
  record_ = record;

  g = 9.807;  // should g be defined here?

  u_.setZero(model.n_a);
  L_ = model.leg_dim;
  h0_ = model.h0;
  J_ = model.inertia;
  mu_ = model.mu;

  n_X = 13;
  n_U = 6;
  X_0.resize(n_X);
  X_0 << 0, 0, h0_, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  X_f.resize(n_X);
  X_f << 2.5, 0, h0_, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  t_p = 0.8;                         // gait period, seconds
  phi_switch = 0.5;                  // switching phase, must be between 0 and 1. Percentage of gait spent in contact.
  N = 40;                            // mpc prediction horizon length (mpc steps)
  dt_mpc = 0.01;                     // mpc sampling time (s), needs to be a factor of N
  N_mpc = dt_mpc / dt_;              // mpc sampling time (timesteps), repeat mpc every x timesteps
  t_horizon = N * dt_mpc;            // mpc horizon time
  N_k = N * N_mpc;                   // total mpc prediction horizon length (low-level timesteps)
  t_start = 0.5 * t_p * phi_switch;  // start halfway through stance phase
  t_st = t_p * phi_switch;           // time spent in stance
  N_c = t_st / dt;                   // number of timesteps spent in contact

  // class definitions
  bridgePtr_.reset(new MujocoBridge(model, dt, g, mu_, fixed, record));
  legPtr_.reset(new Leg(model, dt, g));

};  // constructor

void Runner::Run() {  // Method/function defined inside the class
  bridgePtr_->Init();
  for (int k = 0; k < N_run_; k++) {
    std::cout << k << "\n";
    bridgePtr_->SimRun();  // X, qa, dqa, c, tau, i, v, grf = self.simulator.sim_run(u=self.u)  # run sim
    // legPtr_->UpdateState(a_in, Q_base);
    // u = ctrlPtr_->OpSpacePosCtrl();
  }
};