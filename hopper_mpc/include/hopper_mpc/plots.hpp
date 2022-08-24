#pragma once

#include <Eigen/Dense>

class Plots {
 public:
  Plots();
  // static Eigen::Matrix<int, 4, 3> H;
  // static Eigen::Matrix<int, 4, 4> T;
  static std::vector<double> GenRange(int N);
  static void Theta(int N, std::vector<double> theta_x, std::vector<double> theta_y, std::vector<double> theta_z,
                    std::vector<double> setp_x, std::vector<double> setp_y,
                    std::vector<double> setp_z);  // this function returns yaw angle within -pi to pi
  static void JointPos(int N, std::vector<double> q0, std::vector<double> q2, std::vector<double> q0_ref, std::vector<double> q2_ref);
  static void OpSpacePos(int N, std::vector<double> peb_x, std::vector<double> peb_z, std::vector<double> peb_refx,
                         std::vector<double> peb_refz);
};
