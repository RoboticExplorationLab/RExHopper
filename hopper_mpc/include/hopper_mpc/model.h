#pragma once
#include "Eigen/Dense"

struct Model {
  std::string name;
  std::string csvpath;
  std::string urdfpath;
  std::string aname[5];
  int n_a;
  int s_pol;
  double m;  // leg masses
  double h0;
  double K_s;
  double K;
  double mu;
  Eigen::Vector4d init_q;
  Eigen::Vector4d init_dq;
  Eigen::Vector4d cl;
  Eigen::Matrix<double, 6, 1> leg_dim;  // leg dimensions
  Eigen::Matrix<double, 5, 1> a_kt;
  Eigen::Vector3d rh;
  Eigen::Matrix3d inertia;
  Eigen::Matrix<double, 7, 5> S;  // actuator selection matrix
};