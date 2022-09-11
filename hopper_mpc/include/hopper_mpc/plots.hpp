#pragma once

#include <Eigen/Dense>

class Plots {
 public:
  Plots();
  static void Grf(int N, std::vector<double> grf_normal);
  static void OpSpacePos(int N, std::vector<double> peb_x, std::vector<double> peb_z, std::vector<double> peb_refx,
                         std::vector<double> peb_refz);
  static void Plot2(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                    std::vector<double> vec2, std::vector<double> ref2, double ylim);
  static void Plot3(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                    std::vector<double> vec2, std::vector<double> ref2, std::string name_3, std::vector<double> vec3,
                    std::vector<double> ref3, double ylim);
  static void Plot5(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                    std::vector<double> vec2, std::vector<double> ref2, std::string name_3, std::vector<double> vec3,
                    std::vector<double> ref3, std::string name_4, std::vector<double> vec4, std::vector<double> ref4, std::string name_5,
                    std::vector<double> vec5, std::vector<double> ref5, double ylim);

 private:
  static std::vector<double> GenRange(int N);
};
