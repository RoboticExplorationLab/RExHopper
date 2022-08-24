//
// Created by shuoy on 10/19/21.
// Modified by bbokser starting on 06/26/22.
//
#include "hopper_mpc/plots.hpp"
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/matplotlibcpp.h"

namespace plt = matplotlibcpp;

Plots::Plots(){};

std::vector<double> Plots::GenRange(int N) {
  std::vector<double> timesteps(N);
  for (double i = 0; i < N; i++) {
    timesteps.at(i) = i;
  }
  return timesteps;
}

void Plots::Theta(int N, std::vector<double> theta_x, std::vector<double> theta_y, std::vector<double> theta_z, std::vector<double> setp_x,
                  std::vector<double> setp_y, std::vector<double> setp_z) {
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);

  plt::figure_size(1200, 780);
  plt::suptitle("Theta vs Timesteps");

  plt::subplot(3, 1, 1);
  plt::named_plot("Theta_x", timesteps, theta_x, "r");
  plt::named_plot("Setpoint_x", timesteps, setp_x, "g--");

  plt::subplot(3, 1, 2);
  plt::named_plot("Theta_y", timesteps, theta_y, "r");
  plt::named_plot("Setpoint_y", timesteps, setp_y, "g--");

  plt::subplot(3, 1, 3);
  plt::named_plot("Theta_z", timesteps, theta_z, "r");
  plt::named_plot("Setpoint_z", timesteps, setp_z, "g--");
  plt::legend();
  plt::show();
};

void Plots::JointPos(int N, std::vector<double> q0, std::vector<double> q2, std::vector<double> q0_ref, std::vector<double> q2_ref) {
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);

  plt::figure_size(1200, 780);
  plt::suptitle("Leg Pos vs Timesteps");

  plt::subplot(2, 1, 1);
  plt::named_plot("q2", timesteps, q0, "r");
  plt::named_plot("q2_ref", timesteps, q0_ref, "g--");

  plt::subplot(2, 1, 2);
  plt::named_plot("q2", timesteps, q2, "r");
  plt::named_plot("q2_ref", timesteps, q2_ref, "g--");

  plt::legend();
  plt::show();
};

void Plots::OpSpacePos(int N, std::vector<double> peb_x, std::vector<double> peb_z, std::vector<double> peb_refx,
                       std::vector<double> peb_refz) {
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);

  plt::figure_size(1200, 780);
  plt::suptitle("Body Frame End-Effector Pos vs Timesteps");

  plt::subplot(3, 1, 1);
  plt::named_plot("peb_x", timesteps, peb_x, "r");
  plt::named_plot("peb_refx", timesteps, peb_refx, "g--");

  plt::subplot(3, 1, 2);
  plt::named_plot("peb_z", timesteps, peb_z, "r");
  plt::named_plot("peb_refz", timesteps, peb_refz, "g--");

  plt::subplot(3, 1, 3);
  plt::named_plot("2D Pos", peb_x, peb_z, "ro ");
  plt::named_plot("2D Ref Pos", peb_refx, peb_refz, "go ");

  plt::legend();
  plt::show();
};
