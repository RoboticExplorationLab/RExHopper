#include <iostream>
#include <string>
#include "Eigen/Dense"
#include "hopper_mpc/argparse.hpp"
#include "hopper_mpc/runner.h"
// include "yaml-cpp/yaml.h"

Runner::Runner(Model model, int N_run, double dt, std::string ctrl, bool plot, bool fixed, bool spr, bool record, bool recalc) {
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

};  // constructor

void Runner::Run() {  // Method/function defined inside the class
  for (int k = 0; k < N_run_; k++) {
    std::cout << k << "\n";
    // X, qa, dqa, c, tau, i, v, grf = self.simulator.sim_run(u=self.u)  # run sim
  }
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

  auto ctrl = program.get<std::string>("ctrl");
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
  hopper.aname[4] = "rw3";

  hopper.n_a = 5;
  hopper.spol = 1;
  hopper.h0 = 0.27;
  hopper.ks = 996;
  hopper.kwbc = 5000;
  hopper.mu = 0.5;
  hopper.init_q.resize(4);  // four leg links
  hopper.init_q << -30 * M_PI / 180, -120 * M_PI / 180, -150 * M_PI / 180, 120 * M_PI / 180;
  hopper.linklen.resize(6);
  hopper.linklen << .1, .27, .27, .1, .17, .0205;
  hopper.a_kt.resize(hopper.n_a);
  hopper.a_kt << 1.73, 1.73, 0.106, 0.106, 0.0868;
  hopper.inertia << 0.07542817, 0.00016327, 0.00222099,  // clang-format off
                    0.00016327, 0.04599064,  -0.00008321,
                    0.00222099, -0.00008321, 0.07709692;  // clang-format on
  hopper.S.resize(7, hopper.n_a);
  hopper.S << 1, 0, 0, 0, 0,  // clang-format off
              0, 0, 0, 0, 0, 
              0, 1, 0, 0, 0, 
              0, 0, 0, 0, 0, 
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0, 
              0, 0, 0, 0, 1;  // clang-format on

  double dt = 0.001;
  Runner runner(hopper, N_run, dt, ctrl, plot, fixed, spr, record, false);
  runner.Run();  // Call the method
  return 0;
}
