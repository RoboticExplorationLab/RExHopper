//
// Originally created by shuoy on 10/19/21.
// Modified by bbokser starting on 06/26/22.
//

#pragma once

#include <Eigen/Dense>

class Utils {
 public:
  Utils();
  // static Eigen::Matrix<int, 4, 3> H;
  // static Eigen::Matrix<int, 4, 4> T;
  static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);  // this function returns yaw angle within -pi to pi
  static Eigen::Matrix3d skew(Eigen::Vector3d vec);
  static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d& mat);
  // static Eigen::Matrix4d L(Eigen::Quaterniond Q);
  // static Eigen::Matrix4d R(Eigen::Quaterniond Q);
  // static Eigen::Quaterniond Q_inv(Eigen::Quaterniond Q);
  // static Eigen::Vector3d Z(Eigen::Vector3d vec, Eigen::Quaterniond Q);

};  // namespace Utils

// Eigen::Matrix<int, 4, 3> Utils::H((Eigen::MatrixXd() << 0, 0, 0,  // clang-format off
//                                                         1, 0, 0,
//                                                         0, 1, 0,
//                                                         0, 0, 1).finished());  // clang-format on
// Eigen::Matrix<int, 4, 4> Utils::T((Eigen::MatrixXd() << 1, 0, 0, 0,  // clang-format off
//                                                         0, -1, 0, 0,
//                                                         0, 0, -1, 0,
//                                                         0, 0, 0, -1).finished());  // clang-format on
