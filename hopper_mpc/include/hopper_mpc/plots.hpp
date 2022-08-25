#pragma once

#include <Eigen/Dense>

class Plots {
 public:
  Plots();
  static void Theta(int N, std::vector<double> theta_x, std::vector<double> theta_y, std::vector<double> theta_z,
                    std::vector<double> setp_x, std::vector<double> setp_y,
                    std::vector<double> setp_z);  // this function returns yaw angle within -pi to pi
  static void JointPos(int N, std::vector<double> q0, std::vector<double> q2, std::vector<double> q0_ref, std::vector<double> q2_ref);
  static void OpSpacePos(int N, std::vector<double> peb_x, std::vector<double> peb_z, std::vector<double> peb_refx,
                         std::vector<double> peb_refz);
  static void Tau(int N, std::vector<double> tau_0, std::vector<double> tau_1, std::vector<double> tau_2, std::vector<double> tau_3,
                  std::vector<double> tau_4, std::vector<double> tau_ref0, std::vector<double> tau_ref1, std::vector<double> tau_ref2,
                  std::vector<double> tau_ref3, std::vector<double> tau_ref4);

 private:
  static std::vector<double> GenRange(int N);

  static void Plot3(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                    std::vector<double> vec2, std::vector<double> ref2, std::string name_3, std::vector<double> vec3,
                    std::vector<double> ref3, double ylim);
  static void Plot5(int N, std::string title, std::string name_1, std::vector<double> vec1, std::vector<double> ref1, std::string name_2,
                    std::vector<double> vec2, std::vector<double> ref2, std::string name_3, std::vector<double> vec3,
                    std::vector<double> ref3, std::string name_4, std::vector<double> vec4, std::vector<double> ref4, std::string name_5,
                    std::vector<double> vec5, std::vector<double> ref5, double ylim);
};
