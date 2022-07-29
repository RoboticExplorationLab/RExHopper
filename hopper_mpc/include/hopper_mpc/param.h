#pragma once

#include <string>
#include "Eigen/Dense"

struct {
  std::string name;
  std::string csvpath;
  std::string urdfpath;
  std::string aname[5];
  int n_a;
  int s_pol;
  double h0;
  double ks;
  double kwbc;
  double mu;
  Eigen::Vector4d init_q;
  Eigen::VectorXd linklen;
  Eigen::VectorXd a_kt;
  Eigen::Vector3d rh;
  Eigen::Matrix3d inertia;
  Eigen::MatrixXd S;
} Model;