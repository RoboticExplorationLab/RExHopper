#pragma once
#include <Eigen/Dense>

class LowPass {
 public:
  LowPass(double dt_, double bandwidth_);
  double Filter(double input);

 private:
  double input_smoothed;
  double alpha;
};

class LowPass3D {
 public:
  LowPass3D(double dt_, double bandwidth_);
  Eigen::Vector3d Filter(Eigen::Vector3d input);

 private:
  Eigen::Vector3d input_smoothed;
  double alpha;  // Switch to multidim alpha?
};