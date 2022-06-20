#pragma once
#include "Eigen/Dense"

struct Model {
  std::string name;
  std::string csvpath;
  std::string urdfpath;
  std::string aname[5];
  int n_a;
  int spol;
  double h0;
  double ks;
  double kwbc;
  double mu;
  Eigen::VectorXd init_q;
  Eigen::VectorXd linklen;
  Eigen::VectorXd a_kt;
  Eigen::Vector3d rh;
  Eigen::Matrix3d inertia;
  Eigen::MatrixXd S;
};

class Runner {  // The class
 public:        // Access specifier
  Runner(Model model, int N_run, double dt, std::string ctrl, bool plot, bool fixed, bool spr, bool record, bool recalc);  // constructor

  void Run();

  int n_X;
  int n_U;
  Eigen::VectorXd X_0;
  Eigen::VectorXd X_f;
  Eigen::VectorXd u_;

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

  int N_run_;
  double dt_;
  std::string ctrl_;
  bool plot_;
  bool fixed_;
  bool spr_;
  bool record_;

  Eigen::VectorXd L_;
  double h0_;
  Eigen::Matrix3d J_;
  double mu_;
};