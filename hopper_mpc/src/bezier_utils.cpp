//
// Created by shuoy on 10/19/21.
//

#include "hopper_mpc/bezier_utils.h"

Eigen::Vector3d BezierUtils::get_foot_pos_curve(float t, Eigen::Vector3d foot_pos_start, Eigen::Vector3d foot_pos_final,
                                                double terrain_pitch_angle = 0) {
  Eigen::Vector3d foot_pos_target;
  // X-axis
  std::vector<double> bezierX{foot_pos_start(0), foot_pos_start(0), foot_pos_final(0), foot_pos_final(0), foot_pos_final(0)};
  foot_pos_target(0) = bezier_curve(t, bezierX);

  // Y-axis
  std::vector<double> bezierY{foot_pos_start(1), foot_pos_start(1), foot_pos_final(1), foot_pos_final(1), foot_pos_final(1)};
  foot_pos_target(1) = bezier_curve(t, bezierY);

  // Z-axis
  std::vector<double> bezierZ{foot_pos_start(2), foot_pos_start(2), foot_pos_final(2), foot_pos_final(2), foot_pos_final(2)};
  bezierZ[1] += FOOT_SWING_CLEARANCE1;
  bezierZ[2] += FOOT_SWING_CLEARANCE2 + 0.5 * sin(terrain_pitch_angle);
  foot_pos_target(2) = bezier_curve(t, bezierZ);

  return foot_pos_target;
}