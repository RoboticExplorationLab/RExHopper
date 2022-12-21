// #include <algorithm>
#include <iostream>
#include <string>
#include "Eigen/Dense"
#include "hopper_mpc/argparse.hpp"
#include "hopper_mpc/runner.h"

void checklist(std::vector<std::string> list_in, std::string input, std::string name) {
  if (std::find(std::begin(list_in), std::end(list_in), input) == std::end(list_in)) {
    throw std::runtime_error("Invalid " + name + ": " + input);
  }
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("Hopper");

  program.add_argument("bridge").help("hardware or mujoco");

  program.add_argument("start").help("start_sit, start_stand, or fixed");

  program.add_argument("ctrl").help("mpc, raibert, stand, idle, rotorspeed, or circle");

  program.add_argument("N_run").help("number of timesteps the sim runs for").scan<'i', int>();

  program.add_argument("--plot").help("enable plotting").default_value(false).implicit_value(true);

  program.add_argument("--skip_homing")
      .help("skip homing leg positions (if robot is sitting and already correctly homed)")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--skip_kf").help("ignore kalman filter").default_value(false).implicit_value(true);

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  std::string bridge = program.get<std::string>("bridge");
  std::cout << "bridge = " << (bridge) << std::endl;
  std::string start = program.get<std::string>("start");
  std::cout << "start = " << (start) << std::endl;
  std::string ctrl = program.get<std::string>("ctrl");
  std::cout << "ctrl = " << (ctrl) << std::endl;
  int N_run = program.get<int>("N_run");
  std::cout << "N_run = " << (N_run) << std::endl;

  bool plot = false;
  bool skip_homing = false;
  bool skip_kf = false;

  if (program["--plot"] == true) {
    std::cout << "Plotting enabled" << std::endl;
    plot = true;
  }
  if (program["--skip_homing"] == true) {
    std::cout << "Homing enabled" << std::endl;
    skip_homing = true;
  }
  if (program["--skip_kf"] == true) {
    std::cout << "Ignoring Kalman filter" << std::endl;
    skip_kf = true;
  }

  // basic checks
  std::vector<std::string> bridge_list{"hardware", "mujoco"};
  checklist(bridge_list, bridge, "bridge");

  std::vector<std::string> start_list{"start_stand", "start_sit", "fixed"};
  checklist(start_list, start, "start");

  std::vector<std::string> ctrl_list{"mpc", "raibert", "stand", "idle", "rotorspeed", "circle"};
  checklist(ctrl_list, ctrl, "ctrl");

  if (start != "fixed" && (ctrl == "circle" || ctrl == "rotorspeed")) {
    throw std::runtime_error("Invalid command combination: Cannot run this ctrl test unless robot is fixed in place");
  }

  // setup params
  Model hopper;
  // hopper.name = "rev09";
  // hopper.mjcf_path = "/src/RExHopper/hopper_mpc/res/hopper_rev09/hopper_rev09_mjcf.xml";
  // hopper.mjcf_fixed_path = "/src/RExHopper/hopper_mpc/res/hopper_rev09/hopper_rev09_mjcf_fixed.xml";
  hopper.name = "rev10";
  hopper.mjcf_path = "/src/RExHopper/hopper_mpc/res/hopper_rev10/hopper_rev10_mjcf.xml";
  hopper.mjcf_fixed_path = "/src/RExHopper/hopper_mpc/res/hopper_rev10/hopper_rev10_mjcf_fixed.xml";
  hopper.aname[0] = "q0";
  hopper.aname[1] = "q2";
  hopper.aname[2] = "rw1";
  hopper.aname[3] = "rw2";
  hopper.aname[4] = "rw3";
  hopper.n_a = 5;
  hopper.s_pol = 1;
  hopper.h0 = 0.27;             // leg height in default configuration
  hopper.p0 << 0, 0, 0.50;      // default starting position
  hopper.p0_sit << 0, 0, 0.30;  // default starting position for sitting
  hopper.K = 5000;
  hopper.mu = 0.5;
  hopper.g = 9.807;
  hopper.m << 0.585808, 0.0809766, 0.734902, 0.108053;  // TODO: scrape from mjcf?
  hopper.q_init << -30 * M_PI / 180, -120 * M_PI / 180, -150 * M_PI / 180, 120 * M_PI / 180, 0, 0, 0;
  hopper.dq_init << 0, 0, 0, 0, 0, 0, 0;

  // TODO: Update these values
  hopper.l_c0 << 0.00532254792641475, 0.0312919315403303, 0.000492239493659961;  // leg link CoM positions
  hopper.l_c1 << 0.133101645966398, 0.00662597443711556, 0;
  hopper.l_c2 << 0.0181413414698164, -0.0300644982791918, -0.00029245641838564;
  hopper.l_c3 << 0.101438164997463, -0.00250998180028712, -0.0148532863907363;
  hopper.I << 0.00083862, 0.00074326, 0.00280670, 0.00135609;

  hopper.leg_dim << .1, .27, .27, .1, .17, .0205;
  hopper.a_kt << 1.73, 1.73, 0.106, 0.106, 0.0868;
  hopper.a_tau_stall << 50, 50, 12, 12, 4;                                                       // actuator rated stall torques
  hopper.inertia << 0.07542817, 0.00016327, 0.00222099,  // clang-format off
                    0.00016327, 0.04599064, -0.00008321,
                    0.00222099, -0.00008321, 0.07709692;                  // clang-format on
  hopper.S << 1, 0, 0, 0, 0,  // clang-format off
              0, 0, 0, 0, 0, 
              0, 1, 0, 0, 0, 
              0, 0, 0, 0, 0, 
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0, 
              0, 0, 0, 0, 1;  // clang-format on
  hopper.qla_home << 29 * M_PI / 180, -187 * M_PI / 180;                                         // homing hardstop locations
  hopper.qla_sit << 0 * M_PI / 180, -140 * M_PI / 180;                                           // sitting position
  hopper.qla_stand << -0.886343, -2.13709;                                                       // standing position
  hopper.k_kin << 45, 45 * 0.02;                                                                 // kinematic PD gains
  hopper.N_getup = 500;  // number of timesteps taken to stand up from sitting
  double dt = 0.001;     // 1 kHz

  Runner runner(hopper, dt, bridge, start, ctrl, N_run, plot, skip_homing, skip_kf);
  runner.Run();  // Call the method
  return 0;
}
