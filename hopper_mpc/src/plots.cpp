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
  plt::named_plot("2D Pos", peb_x, peb_z, "r");
  plt::named_plot("2D Ref Pos", peb_refx, peb_refz, "go ");
  plt::xlim(-1, 1);
  plt::ylim(-0.7, 0.0);

  plt::legend();
  plt::show();
};

void Plots::Plot2(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                  std::vector<double> vec2, std::vector<double> ref2, double ylim) {
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);

  plt::figure_size(1200, 780);
  plt::suptitle(title);

  plt::subplot(2, 1, 1);
  plt::named_plot(name_1, timesteps, vec1, "r");
  plt::named_plot(name_1 + " Ref", timesteps, ref1, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::subplot(2, 1, 2);
  plt::named_plot(name_2, timesteps, vec2, "r");
  plt::named_plot(name_2 + " Ref", timesteps, ref2, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::legend();
  plt::show();
};

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
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::subplot(3, 1, 2);
  plt::named_plot(name_2, timesteps, vec2, "r");
  plt::named_plot(name_2 + " Ref", timesteps, ref2, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::subplot(3, 1, 3);
  plt::named_plot(name_3, timesteps, vec3, "r");
  plt::named_plot(name_3 + " Ref", timesteps, ref3, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::legend();
  plt::show();
};

void Plots::Plot5(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                  std::vector<double> vec2, std::vector<double> ref2, std::string name_3, std::vector<double> vec3,
                  std::vector<double> ref3, std::string name_4, std::vector<double> vec4, std::vector<double> ref4, std::string name_5,
                  std::vector<double> vec5, std::vector<double> ref5, double ylim) {
  // if ylim = 0, use autoscaling
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);

  plt::figure_size(1200, 780);
  plt::suptitle(title);

  plt::subplot(5, 1, 1);
  plt::named_plot(name_1, timesteps, vec1, "r");
  plt::named_plot(name_1 + " Ref", timesteps, ref1, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::subplot(5, 1, 2);
  plt::named_plot(name_2, timesteps, vec2, "r");
  plt::named_plot(name_2 + " Ref", timesteps, ref2, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::subplot(5, 1, 3);
  plt::named_plot(name_3, timesteps, vec3, "r");
  plt::named_plot(name_3 + " Ref", timesteps, ref3, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::subplot(5, 1, 4);
  plt::named_plot(name_4, timesteps, vec4, "r");
  plt::named_plot(name_4 + " Ref", timesteps, ref4, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::subplot(5, 1, 5);
  plt::named_plot(name_5, timesteps, vec5, "r");
  plt::named_plot(name_5 + " Ref", timesteps, ref5, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }

  plt::legend();
  plt::show();
};
