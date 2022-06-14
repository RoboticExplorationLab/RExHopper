#include <iostream>
#include <string>
#include "Eigen/Dense"
#include "hopper_mpc/argparse.hpp"
// include "yaml-cpp/yaml.h"

using namespace std;

struct Model {
  string name;
  string csvpath;
  string urdfpath;
  string aname[4];
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Runner(Model model, int N_run, double dt, string ctrl, bool plot, bool fixed, bool spr, bool record, bool recalc) {
    N_run_ = N_run;
    dt_ = dt;
    ctrl_ = ctrl;
    plot_ = plot;
    fixed_ = fixed;
    spr_ = spr;
    record_ = record;

    u_.setZero(model.n_a);
    L_ = model.linklen;
    h0_ = model.h0;
    J_ = model.inertia;
    mu_ = model.mu;

    n_X = 13;
    n_U = 6;
    X_0.resize(13);
    X_0 << 0, 0, h0_, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    X_f.resize(13);
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

  };  // constructor

  void Run() {  // Method/function defined inside the class
    for (int k = 0; k < N_run_; k++) {
      cout << k << "\n";
      // X, qa, dqa, c, tau, i, v, grf = self.simulator.sim_run(u=self.u)  # run sim
    }
  };

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
  string ctrl_;
  bool plot_;
  bool fixed_;
  bool spr_;
  bool record_;

  Eigen::VectorXd L_;
  double h0_;
  Eigen::Matrix3d J_;
  double mu_;
};

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("Hopper");

  program.add_argument("ctrl").help("mpc, wbc_raibert, wbc_vert, or wbc_static");

  program.add_argument("N_run").help("number of timesteps the sim runs for").scan<'i', int>();

  program.add_argument("--plot").help("enable plotting").default_value(false).implicit_value(true);

  program.add_argument("--fixed").help("fix the robot in place").default_value(false).implicit_value(true);

  program.add_argument("--spr").help("add parallel spring").default_value(false).implicit_value(true);

  program.add_argument("--record").help("record video").default_value(false).implicit_value(true);

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  auto ctrl = program.get<string>("ctrl");
  std::cout << (ctrl) << std::endl;
  auto N_run = program.get<int>("N_run");
  std::cout << (N_run) << std::endl;

  bool plot = false;
  bool spr = false;
  bool fixed = false;
  bool record = false;

  if (program["--plot"] == true) {
    std::cout << "Plotting enabled" << std::endl;
    plot = true;
  }
  if (program["--spr"] == true) {
    std::cout << "Spring enabled" << std::endl;
    spr = true;
  }
  if (program["--fixed"] == true) {
    std::cout << "Fixed base enabled" << std::endl;
    fixed = true;
  }
  if (program["--record"] == true) {
    std::cout << "Recording enabled" << std::endl;
    record = true;
  }

  // auto input = yaml.parse(program.get("filename"))
  // model.mass = input["mass"]
  // model.csvpath = input["csvpath"]
  Model hopper;
  hopper.name = "rw";
  hopper.csvpath = "res/hopper_rev08/urdf/hopper_rev08.csv";
  hopper.urdfpath = "res/hopper_rev08/urdf/hopper_rev08.urdf";
  hopper.aname[0] = "q0";
  hopper.aname[1] = "q2";
  hopper.aname[2] = "rw1";
  hopper.aname[3] = "rw2";

  hopper.n_a = 5;
  hopper.spol = 1;
  hopper.h0 = 0.27;
  hopper.ks = 996;
  hopper.kwbc = 5000;
  hopper.mu = 0.5;
  hopper.init_q << -30 * M_PI / 180, -120 * M_PI / 180, -150 * M_PI / 180, 120 * M_PI / 180;
  hopper.linklen << .1, .27, .27, .1, .17, .0205;
  hopper.inertia << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  hopper.S << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

  double dt = 0.001;
  Runner runner(hopper, N_run, dt, ctrl, plot, fixed, spr, record, false);
  runner.Run();  // Call the method
  return 0;
}