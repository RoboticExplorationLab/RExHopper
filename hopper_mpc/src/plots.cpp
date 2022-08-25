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
  Plot3(N, "Theta vs Timesteps", "Theta_x", theta_x, setp_x, "Theta_y", theta_y, setp_y, "Theta_z", theta_z, setp_z, M_PI);
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

void Plots::Tau(int N, std::vector<double> tau_0, std::vector<double> tau_1, std::vector<double> tau_2, std::vector<double> tau_3,
                std::vector<double> tau_4, std::vector<double> tau_ref0, std::vector<double> tau_ref1, std::vector<double> tau_ref2,
                std::vector<double> tau_ref3, std::vector<double> tau_ref4) {
  Plot5(N, "Tau vs Timesteps", "Tau_0", tau_0, tau_ref0, "Tau_1", tau_1, tau_ref1, "Tau_2", tau_2, tau_ref2, "Tau_3", tau_3, tau_ref3,
        "Tau_4", tau_4, tau_ref4, 60);
}

void Plots::Plot3(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                  std::vector<double> vec2, std::vector<double> ref2, std::string name_3, std::vector<double> vec3,
                  std::vector<double> ref3, double ylim) {
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);

  plt::figure_size(1200, 780);
  plt::suptitle(title);

  plt::subplot(3, 1, 1);
  plt::named_plot(name_1, timesteps, vec1, "r");
  plt::named_plot(name_1 + " Ref", timesteps, ref1, "g--");
  plt::ylim(-ylim, ylim);

  plt::subplot(3, 1, 2);
  plt::named_plot(name_2, timesteps, vec2, "r");
  plt::named_plot(name_2 + " Ref", timesteps, ref2, "g--");
  plt::ylim(-ylim, ylim);

  plt::subplot(3, 1, 3);
  plt::named_plot(name_3, timesteps, vec3, "r");
  plt::named_plot(name_3 + " Ref", timesteps, ref3, "g--");
  plt::ylim(-ylim, ylim);

  plt::legend();
  plt::show();
};

void Plots::Plot5(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                  std::vector<double> vec2, std::vector<double> ref2, std::string name_3, std::vector<double> vec3,
                  std::vector<double> ref3, std::string name_4, std::vector<double> vec4, std::vector<double> ref4, std::string name_5,
                  std::vector<double> vec5, std::vector<double> ref5, double ylim) {
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);

  plt::figure_size(1200, 780);
  plt::suptitle(title);

  plt::subplot(5, 1, 1);
  plt::named_plot(name_1, timesteps, vec1, "r");
  plt::named_plot(name_1 + " Ref", timesteps, ref1, "g--");
  plt::ylim(-ylim, ylim);

  plt::subplot(5, 1, 2);
  plt::named_plot(name_2, timesteps, vec2, "r");
  plt::named_plot(name_2 + " Ref", timesteps, ref2, "g--");
  plt::ylim(-ylim, ylim);

  plt::subplot(5, 1, 3);
  plt::named_plot(name_3, timesteps, vec3, "r");
  plt::named_plot(name_3 + " Ref", timesteps, ref3, "g--");
  plt::ylim(-ylim, ylim);

  plt::subplot(5, 1, 4);
  plt::named_plot(name_4, timesteps, vec4, "r");
  plt::named_plot(name_4 + " Ref", timesteps, ref4, "g--");
  plt::ylim(-ylim, ylim);

  plt::subplot(5, 1, 5);
  plt::named_plot(name_5, timesteps, vec5, "r");
  plt::named_plot(name_5 + " Ref", timesteps, ref5, "g--");
  plt::ylim(-ylim, ylim);

  plt::legend();
  plt::show();
};
