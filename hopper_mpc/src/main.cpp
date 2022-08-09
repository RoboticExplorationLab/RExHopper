#include <iostream>
#include <string>
#include "Eigen/Dense"
#include "hopper_mpc/argparse.hpp"
#include "hopper_mpc/runner.h"
// include "yaml-cpp/yaml.h"

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("Hopper");

  program.add_argument("ctrl").help("mpc, wbc_raibert, wbc_vert, or wbc_static");

  program.add_argument("N_run").help("number of timesteps the sim runs for").scan<'i', int>();

  program.add_argument("bridge").help("hardware, mujoco, or raisim");

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

  std::string ctrl = program.get<std::string>("ctrl");
  std::cout << (ctrl) << std::endl;
  int N_run = program.get<int>("N_run");
  std::cout << (N_run) << std::endl;
  std::string bridge = program.get<std::string>("bridge");
  std::cout << (bridge) << std::endl;

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
  hopper.s_pol = 1;
  hopper.h0 = 0.27;
  hopper.K_s = 996;
  hopper.K = 5000;
  hopper.mu = 0.5;
  hopper.q_init << -30 * M_PI / 180, -120 * M_PI / 180, -150 * M_PI / 180, 120 * M_PI / 180;
  hopper.dq_init << 0, 0, 0, 0;
  hopper.leg_dim << .1, .27, .27, .1, .17, .0205;
  hopper.a_kt << 1.73, 1.73, 0.106, 0.106, 0.0868;
  hopper.inertia << 0.07542817, 0.00016327, 0.00222099,  // clang-format off
                    0.00016327, 0.04599064,  -0.00008321,
                    0.00222099, -0.00008321, 0.07709692;                  // clang-format on
  hopper.S << 1, 0, 0, 0, 0,  // clang-format off
              0, 0, 0, 0, 0, 
              0, 1, 0, 0, 0, 
              0, 0, 0, 0, 0, 
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0, 
              0, 0, 0, 0, 1;  // clang-format on

  double dt = 0.001;
  Runner runner(hopper, N_run, dt, ctrl, bridge, plot, fixed, spr, record);
  runner.Run();  // Call the method
  return 0;
}
