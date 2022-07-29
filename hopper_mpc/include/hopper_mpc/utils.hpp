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
  static double WrapToPi(double a);
};
