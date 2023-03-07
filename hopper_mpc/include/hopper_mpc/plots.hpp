#pragma once

#include <Eigen/Dense>

class Plots {
 public:
  Plots();
  static void PlotSingle(int N, std::string title, std::vector<double> vec);
  static void PlotMulti3(int N, std::string title, std::string name1, std::vector<double> vec1, std::string name2, std::vector<double> vec2,
                         std::string name3, std::vector<double> vec3);
  static void PlotMap2D(int N, std::string title, std::string name, std::vector<std::vector<double>> vec,
                        std::vector<std::vector<double>> ref, double xlim, double ylim);
  static void PlotMap3D(int N, std::string title, std::string name, std::vector<std::vector<double>> vec, double xlim, double ylim);
  static void Plot2(int N, std::string title, std::string name, std::vector<std::vector<double>> vec, std::vector<std::vector<double>> ref,
                    double ylim);
  static void Plot3(int N, std::string title, std::string name, std::vector<std::vector<double>> vec, std::vector<std::vector<double>> ref,
                    double ylim);
  static void Plot5(int N, std::string title, std::string name, std::vector<std::vector<double>> vec, std::vector<std::vector<double>> ref,
                    double ylim);

 private:
  static std::vector<double> StartPlot(int N, std::string title);
  static std::vector<double> StartSubPlot(int N, std::string title);
  static void SubPlot(std::string name, std::vector<double> timesteps, std::vector<double> vec, std::vector<double> ref, double ylim);
  static std::vector<double> ExtractVectorCol(int N, std::vector<std::vector<double>> vec, int col);
  static std::vector<double> ResizeVector(int N, std::vector<double> vec);
  static std::vector<double> GenRange(int N);
};
