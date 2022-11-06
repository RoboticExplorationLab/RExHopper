#include "hopper_mpc/plots.hpp"
#include <iostream>
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

std::vector<double> Plots::StartPlot(int N, std::string title) {
  plt::figure_size(1200, 780);
  plt::title(title);
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);
  return timesteps;
}

std::vector<double> Plots::StartSubPlot(int N, std::string title) {
  plt::figure_size(1200, 780);
  plt::suptitle(title);
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);
  return timesteps;
}

std::vector<double> Plots::ExtractVectorCol(int N, std::vector<std::vector<double>> vec, int col) {
  std::vector<double> vec_col(N);
  int k = 0;
  for (auto i : vec) {
    if (k >= N - 1) {
      break;
    }
    vec_col.at(k) = i.at(col);
    k += 1;
  }
  return vec_col;
}

void Plots::SubPlot(std::string name, std::vector<double> timesteps, std::vector<double> vec, std::vector<double> ref, double ylim) {
  plt::named_plot(name, timesteps, vec, "r");
  plt::named_plot(name + " Ref", timesteps, ref, "g--");
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }
  plt::legend();
}

void Plots::PlotSingle(int N, std::string title, std::vector<double> vec) {
  auto timesteps = StartPlot(N, title);

  plt::plot(timesteps, vec, "r");

  plt::show();
};

void Plots::PlotMulti3(int N, std::string title, std::string name1, std::vector<double> vec1, std::string name2, std::vector<double> vec2,
                       std::string name3, std::vector<double> vec3) {
  auto timesteps = StartSubPlot(N, title);

  plt::subplot(3, 1, 1);
  plt::named_plot(name1, timesteps, vec1, "g");
  plt::legend();

  plt::subplot(3, 1, 2);
  plt::named_plot(name2, timesteps, vec2, "r");
  plt::legend();

  plt::subplot(3, 1, 3);
  plt::named_plot(name3, timesteps, vec3, "b");
  plt::legend();

  plt::show();
};

void Plots::PlotMap2D(int N, std::string title, std::string name, std::vector<std::vector<double>> vec,
                      std::vector<std::vector<double>> ref, double xlim, double ylim) {
  auto timesteps = StartPlot(N, title);

  std::vector<double> vec1(N), vec2(N), ref1(N), ref2(N);
  vec1 = ExtractVectorCol(N, vec, 0);
  vec2 = ExtractVectorCol(N, vec, 2);
  ref1 = ExtractVectorCol(N, ref, 0);
  ref2 = ExtractVectorCol(N, ref, 2);

  plt::plot();
  plt::named_plot(name, vec1, vec2, "r");
  plt::named_plot(name + "_ref", ref1, ref2, "go ");
  if (xlim != 0) {
    plt::xlim(-xlim, xlim);
  }
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }
  plt::legend();

  plt::show();
};

void Plots::PlotMap3D(int N, std::string title, std::string name, std::vector<std::vector<double>> vec, double xlim, double ylim) {
  // matplotlib-cpp is missing the capability to plot multiple functions on the same graph
  // TODO: use this? https://github.com/Jean1995/matplotlib-cpp/commit/14346be71d18173823d327b34ee3c70006969237

  // plt::figure_size(1200, 780);
  // plt::title(title);
  std::vector<double> timesteps(N);
  timesteps = GenRange(N);

  std::vector<double> vec1(N), vec2(N), vec3(N), ref1(N), ref2(N), ref3(N);
  vec1 = ExtractVectorCol(N, vec, 0);
  vec2 = ExtractVectorCol(N, vec, 1);
  vec3 = ExtractVectorCol(N, vec, 2);
  // ref1 = ExtractVectorCol(N, ref, 0);
  // ref2 = ExtractVectorCol(N, ref, 1);
  // ref3 = ExtractVectorCol(N, ref, 2);

  plt::plot3(vec1, vec2, vec3, {{"c", "r"}, {"label", name}});
  // plt::plot3(ref1, ref2, ref3, {{"c", "g"}, {"ls", "--"}, {"label", name + "_ref"}});
  plt::xlabel("x");
  plt::ylabel("y");
  plt::set_zlabel("z");
  if (xlim != 0) {
    plt::xlim(-xlim, xlim);
  }
  if (ylim != 0) {
    plt::ylim(-ylim, ylim);
  }
  plt::legend();

  plt::show();
};

void Plots::Plot2(int N, std::string title, std::string name, std::vector<std::vector<double>> vec, std::vector<std::vector<double>> ref,
                  double ylim) {
  auto timesteps = StartSubPlot(N, title);

  std::vector<double> vec1(N), vec2(N), ref1(N), ref2(N);
  vec1 = ExtractVectorCol(N, vec, 0);
  vec2 = ExtractVectorCol(N, vec, 1);
  ref1 = ExtractVectorCol(N, ref, 0);
  ref2 = ExtractVectorCol(N, ref, 1);

  plt::subplot(2, 1, 1);
  SubPlot(name + "_1", timesteps, vec1, ref1, ylim);

  plt::subplot(2, 1, 2);
  SubPlot(name + "_2", timesteps, vec2, ref2, ylim);

  plt::show();
};

void Plots::Plot3(int N, std::string title, std::string name, std::vector<std::vector<double>> vec, std::vector<std::vector<double>> ref,
                  double ylim) {
  auto timesteps = StartSubPlot(N, title);

  std::vector<double> vec1(N), vec2(N), vec3(N), ref1(N), ref2(N), ref3(N);
  vec1 = ExtractVectorCol(N, vec, 0);
  vec2 = ExtractVectorCol(N, vec, 1);
  vec3 = ExtractVectorCol(N, vec, 2);
  ref1 = ExtractVectorCol(N, ref, 0);
  ref2 = ExtractVectorCol(N, ref, 1);
  ref3 = ExtractVectorCol(N, ref, 2);

  plt::subplot(3, 1, 1);
  SubPlot(name + "_x", timesteps, vec1, ref1, ylim);

  plt::subplot(3, 1, 2);
  SubPlot(name + "_y", timesteps, vec2, ref2, ylim);

  plt::subplot(3, 1, 3);
  SubPlot(name + "_z", timesteps, vec3, ref3, ylim);

  plt::show();
};

void Plots::Plot5(int N, std::string title, std::string name, std::vector<std::vector<double>> vec, std::vector<std::vector<double>> ref,
                  double ylim) {
  auto timesteps = StartSubPlot(N, title);

  std::vector<double> vec1(N), vec2(N), vec3(N), vec4(N), vec5(N), ref1(N), ref2(N), ref3(N), ref4(N), ref5(N);
  vec1 = ExtractVectorCol(N, vec, 0);
  vec2 = ExtractVectorCol(N, vec, 1);
  vec3 = ExtractVectorCol(N, vec, 2);
  vec4 = ExtractVectorCol(N, vec, 3);
  vec5 = ExtractVectorCol(N, vec, 4);

  ref1 = ExtractVectorCol(N, ref, 0);
  ref2 = ExtractVectorCol(N, ref, 1);
  ref3 = ExtractVectorCol(N, ref, 2);
  ref4 = ExtractVectorCol(N, ref, 3);
  ref5 = ExtractVectorCol(N, ref, 4);

  plt::subplot(5, 1, 1);
  SubPlot(name + "_1", timesteps, vec1, ref1, ylim);

  plt::subplot(5, 1, 2);
  SubPlot(name + "_2", timesteps, vec2, ref2, ylim);

  plt::subplot(5, 1, 3);
  SubPlot(name + "_3", timesteps, vec3, ref3, ylim);

  plt::subplot(5, 1, 4);
  SubPlot(name + "_4", timesteps, vec4, ref4, ylim);

  plt::subplot(5, 1, 5);
  SubPlot(name + "_5", timesteps, vec5, ref5, ylim);

  plt::show();
};
