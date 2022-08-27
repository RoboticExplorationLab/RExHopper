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
  static Eigen::Vector3d QuatToEuler(Eigen::Quaterniond quat);  // this function returns yaw angle within -pi to pi
  static Eigen::Matrix3d Skew(Eigen::Vector3d vec);
  static Eigen::Matrix3d PseudoInverse(const Eigen::Matrix3d& mat);
  static Eigen::Quaterniond VecToQuat(Eigen::Vector3d v2);
  static double WrapToPi(double a);
  static double Clip(double n, double lower, double upper);
};
