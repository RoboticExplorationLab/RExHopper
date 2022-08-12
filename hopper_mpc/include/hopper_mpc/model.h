#pragma once
#include "Eigen/Dense"

struct retVals {                    // Declare a local structure
  Eigen::Matrix<double, 13, 1> X;   // simulation state (p, Q, v, omega)
  Eigen::Matrix<double, 5, 1> qa;   // actuated joint positions
  Eigen::Matrix<double, 5, 1> dqa;  // actuated joint velocities
};

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
  Eigen::Vector4d m;                    // leg masses
  Eigen::Matrix<double, 7, 1> q_init;   // default initial joint pos
  Eigen::Matrix<double, 7, 1> dq_init;  // default initial joint vel
  Eigen::Vector3d l_c0;                 // leg link CoM positions
  Eigen::Vector3d l_c1;
  Eigen::Vector3d l_c2;
  Eigen::Vector3d l_c3;
  Eigen::Vector4d I;                    // leg link moments of inertia
  Eigen::Matrix<double, 6, 1> leg_dim;  // leg dimensions
  Eigen::Matrix<double, 5, 1> a_kt;     // actuator KT ratings
  Eigen::Vector3d rh;
  Eigen::Matrix3d inertia;        // total inertia matrix
  Eigen::Matrix<double, 7, 5> S;  // actuator selection matrix

  // Note: q and a vectors will have many different sizes depending on the situation.
  // This can be confusing.
  // Vectors of length 7 include every movable joint on the robot. General purpose.
  // Vectors of length 5 represent actuatable joints (q0, q2, q4, q5, q6). Used for ctrl vector u.
  // Vectors of length 4 represent just the leg joints (q0, q1, q2, q3). Used in Leg class only.
  // Vectors of length 3 represent just the reaction wheel joints (q4, q5, q6). Used in Rwa class only.
  // Vectors of length 2 represent just the actuatable leg joints (q0, q2). Used for leg jacobian calculations.
};