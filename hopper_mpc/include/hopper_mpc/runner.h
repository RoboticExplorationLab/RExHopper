#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/bridge_mujoco.h"
#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/state_machine.h"

class Runner {  // The class

 public:  // Access specifier
  Runner(Model model, int N_run, double dt, std::string ctrl, bool plot, bool fixed, bool spr, bool record, bool recalc);  // constructor

  void Run();
  Model model;

 private:
  int N_run_;         // number of timesteps in sim
  int N_sit_;         // number of timesteps to "sit" at end of traj
  double dt_;         // timestep size
  std::string ctrl_;  // controller
  bool plot_;
  bool fixed_;
  bool spr_;
  bool record_;
  double g_;  // gravitational constant

  int n_X_;                           // number of sim states
  int n_U_;                           // number of sim controls
  Eigen::Matrix<double, 13, 1> X_0_;  // init state
  Eigen::Matrix<double, 13, 1> X_f_;  // final state
  Eigen::Matrix<double, 5, 1> u_;     // controls

  double t_p_;         // gait period, seconds
  double phi_switch_;  // switching phase, must be between 0 and 1. Percentage of gait spent in contact.
  int N_;              // mpc prediction horizon length (mpc steps)
  double dt_mpc_;      // mpc sampling time (s), needs to be a factor of N
  int N_mpc_;          // mpc sampling time (timesteps), repeat mpc every x timesteps
  double t_horizon_;   // mpc horizon time
  int N_k_;            // total mpc prediction horizon length (low-level timesteps)
  double t_start_;     // start halfway through stance phase
  double t_stance_;    // time spent in stance
  int N_c_;            // number of timesteps spent in contact

  Eigen::VectorXd L_;
  double h0_;
  Eigen::Matrix3d J_;
  double mu_;

  std::unique_ptr<MujocoBridge> bridgePtr_;
  std::unique_ptr<Leg> legPtr_;

  bool ContactSchedule(double t, double t0);
  bool ContactMap(int N, double dt, double ts, double t0);
  Eigen::MatrixXd Runner::RefTraj(Eigen::Matrix<double, 12, 1> x_in, Eigen::Matrix<double, 12, 1> x_f);

  Eigen::MatrixXd x_ref_0_;

  StateMachine stateMachine;
  Update update_;
  void FsmUpdate(bool s, bool sh, double dz);
  bool inCmpr_;
  bool inPush_;
  bool inRise_;
  bool inFall_;
  std::string state_;
  std::string state_prev_;
};