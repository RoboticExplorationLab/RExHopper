//
// Created by shuoy on 10/19/21.
//

#ifndef A1_CPP_UTILS_H
#define A1_CPP_UTILS_H

#include <Eigen/Dense>
#include <iostream>

#include "hopper_mpc/bezier.h"
#include "hopper_mpc/spline.h"

class Utils {
  typedef Eigen::Matrix<double, 4, 4> Matrix4d;

 public:
  // compare to Eigen's default eulerAngles
  // this function returns yaw angle within -pi to pi
  static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
  template <typename T>
  static Eigen::Matrix<T, 3, 3> hat(Eigen::Matrix<T, 3, 1> vec);

  template <typename T>
  static Eigen::Matrix<T, 4, 4> L(Eigen::Quaternion<T> q);
  static Eigen::Matrix<double, 4, 3> H();
  static Eigen::Quaterniond rho(Eigen::Vector3d phi);
};

class BezierUtils {
  // TODO: allow degree change? may not be necessary, we can stick to degree 4
 public:
  BezierUtils() {
    curve_constructed = false;
    bezier_degree = 4;
  }
  // set of functions create bezier curves, get points, reset
  Eigen::Vector3d get_foot_pos_curve(float t, Eigen::Vector3d foot_pos_start, Eigen::Vector3d foot_pos_final);

  bool reset_foot_pos_curve() { curve_constructed = false; }

 private:
  double bezier_curve(double t, const std::vector<double>& P);

  bool curve_constructed;
  float bezier_degree;
};

class CubicSpineUtils {
  // use https://kluge.in-chemnitz.de/opensource/spline/ to generate foot trajectory
 public:
  CubicSpineUtils() { curve_constructed = false; }
  // set of functions create spline curves, get points, reset
  bool set_foot_pos_curve(Eigen::Vector3d foot_pos_start, Eigen::Vector3d foot_pos_final);
  Eigen::Vector3d get_foot_pos_curve(double t);
  Eigen::Vector3d get_foot_vel_curve(double t);
  Eigen::Vector3d get_foot_acc_curve(double t);

  bool reset_foot_pos_curve() { curve_constructed = false; }

 private:
  int num_pts = 3;
  bool curve_constructed;
  tk::spline sp_x;
  tk::spline sp_y;
  tk::spline sp_z;
};

#endif  // A1_CPP_UTILS_H
