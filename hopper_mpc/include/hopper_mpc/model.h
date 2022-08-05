#pragma once
#include "Eigen/Dense"

struct Model {
  std::string name;
  std::string csvpath;
  std::string urdfpath;
  std::string aname[5];
  int n_a;
  int s_pol;
  double h0;
  double K_s;
  double K;
  double mu;
  Eigen::Vector4d m;  // leg masses
  Eigen::Vector4d q_init;
  Eigen::Vector4d dq_init;
  Eigen::Vector3d l_c0;
  Eigen::Vector3d l_c1;
  Eigen::Vector3d l_c2;
  Eigen::Vector3d l_c3;
  Eigen::Vector4d I;                    // leg link moments of inertia
  Eigen::Matrix<double, 6, 1> leg_dim;  // leg dimensions
  Eigen::Matrix<double, 5, 1> a_kt;
  Eigen::Vector3d rh;
  Eigen::Matrix3d inertia;        // total inertia matrix
  Eigen::Matrix<double, 7, 5> S;  // actuator selection matrix
};