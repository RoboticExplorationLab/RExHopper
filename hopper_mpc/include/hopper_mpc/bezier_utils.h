//
// Originally created by shuoy on 10/19/21.
//

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "hopper_mpc/bezier.h"

class BezierUtils {
  // TODO: allow degree change? may not be necessary, we can stick to degree 4
 public:
  BezierUtils() {
    curve_constructed = false;
    bezier_degree = 4;
  }
  // set of functions create bezier curves, get points, reset
  Eigen::Vector3d get_foot_pos_curve(float t, Eigen::Vector3d foot_pos_start, Eigen::Vector3d foot_pos_final, double terrain_pitch_angle);

  bool reset_foot_pos_curve() { curve_constructed = false; }

 private:
  double bezier_curve(double t, const std::vector<double>& P);

  bool curve_constructed;
  float bezier_degree;
};
