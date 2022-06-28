#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/bridge_mujoco.h"
#include "hopper_mpc/ctrl.h"
#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"

class Runner {  // The class
 public:        // Access specifier
  Runner(Model model, int N_run, double dt, std::string ctrl, bool plot, bool fixed, bool spr, bool record, bool recalc);  // constructor

  void Run();

  int N_run_;         // number of timesteps in sim
  double dt_;         // timestep size
  std::string ctrl_;  // controller
  bool plot_;
  bool fixed_;
  bool spr_;
  bool record_;
  double g;  // gravitational constant

  int n_X;              // number of sim states
  int n_U;              // number of sim controls
  Eigen::VectorXd X_0;  // init state
  Eigen::VectorXd X_f;  // final state
  Eigen::VectorXd u_;   // controls

  double t_p;         // gait period, seconds
  double phi_switch;  // switching phase, must be between 0 and 1. Percentage of gait spent in contact.
  int N;              // mpc prediction horizon length (mpc steps)
  double dt_mpc;      // mpc sampling time (s), needs to be a factor of N
  int N_mpc;          // mpc sampling time (timesteps), repeat mpc every x timesteps
  double t_horizon;   // mpc horizon time
  int N_k;            // total mpc prediction horizon length (low-level timesteps)
  double t_start;     // start halfway through stance phase
  double t_st;        // time spent in stance
  int N_c;            // number of timesteps spent in contact

  Eigen::VectorXd L_;
  double h0_;
  Eigen::Matrix3d J_;
  double mu_;

  std::unique_ptr<MujocoBridge> bridgePtr_;
  std::unique_ptr<Control> ctrlPtr_;
  std::unique_ptr<Leg> legPtr_;
};