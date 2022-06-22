#pragma once
#include "Eigen/Dense"

struct Model {
  std::string name;
  std::string csvpath;
  std::string urdfpath;
  std::string aname[5];
  int n_a;
  int spol;
  double h0;
  double ks;
  double kwbc;
  double mu;
  Eigen::VectorXd init_q;
  Eigen::VectorXd linklen;
  Eigen::VectorXd a_kt;
  Eigen::Vector3d rh;
  Eigen::Matrix3d inertia;
  Eigen::MatrixXd S;
};