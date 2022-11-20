#pragma once

#include <Eigen/Dense>

class Utils {
 public:
  Utils();
  // static Eigen::Matrix<int, 4, 3> H;
  // static Eigen::Matrix<int, 4, 4> T;
  static Eigen::Vector3d QuatToEuler(const Eigen::Quaterniond quat);  // this function returns yaw angle within -pi to pi
  static Eigen::Quaterniond EulerToQuat(const double roll, const double pitch, const double yaw);
  static Eigen::Quaterniond VecToQuat(Eigen::Vector3d v1, Eigen::Vector3d v2);
  static Eigen::Quaterniond GenYawQuat(const double z_angle);
  static Eigen::Quaterniond ExtractYawQuat(Eigen::Quaterniond Q);
  static Eigen::Matrix3d Skew(Eigen::Vector3d vec);
  static Eigen::Matrix3d PseudoInverse(const Eigen::Matrix3d& mat);

  static double AngleBetween(Eigen::Quaterniond Q1, Eigen::Quaterniond Q2);
  static double WrapToPi(double a);
  static double Clip(double n, double lower, double upper);
  static double PolyFit(const std::vector<double>& t, const std::vector<double>& v, int k, double t_new);
};
