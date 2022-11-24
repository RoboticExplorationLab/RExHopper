#pragma once
#include "Eigen/Dense"

struct Model {
  std::string name;
  std::string mjcf_path;
  std::string mjcf_fixed_path;
  std::string aname[5];
  int n_a;
  int s_pol;
  double h0;
  Eigen::Vector3d p0;      // default starting position
  Eigen::Vector3d p0_sit;  // sitting initial position
  double K;
  double mu;                            // coeff of foot-floor friction
  double g;                             // gravitational constant
  Eigen::Vector4d m;                    // leg masses
  Eigen::Matrix<double, 7, 1> q_init;   // default initial joint pos
  Eigen::Matrix<double, 7, 1> dq_init;  // default initial joint vel
  Eigen::Vector3d l_c0;                 // leg link CoM positions
  Eigen::Vector3d l_c1;
  Eigen::Vector3d l_c2;
  Eigen::Vector3d l_c3;
  Eigen::Vector4d I;                        // leg link moments of inertia
  Eigen::Matrix<double, 6, 1> leg_dim;      // leg dimensions
  Eigen::Matrix<double, 5, 1> a_kt;         // actuator KT ratings
  Eigen::Matrix<double, 5, 1> a_tau_stall;  // actuator rated stall torques
  Eigen::Vector3d rh;
  Eigen::Matrix3d inertia;        // total inertia matrix
  Eigen::Matrix<double, 7, 5> S;  // actuator selection matrix
  Eigen::Vector2d qla_home;       // home positions for leg homing
  Eigen::Vector2d qla_sit;        // position for default standing
  Eigen::Vector2d qla_stand;      // position for default standing
  Eigen::Vector2d k_kin;          // inv kin control gains kp and kd
  int N_getup;                    // number of timesteps to get up from sitting position
  // Note: q and a vectors will have many different sizes depending on the situation.
  // This can be confusing.
  // (q) Vectors of length 7 include every movable joint on the robot. General purpose.
  // (qa) Vectors of length 5 represent actuatable joints (q0, q2, q4, q5, q6). Used for ctrl vector u.
  // Vectors of length 4 represent just the leg joints (q0, q1, q2, q3). Used in Leg class only.
  // Vectors of length 3 represent just the reaction wheel joints (q4, q5, q6). Used in Rwa class only.
  // (qla) Vectors of length 2 represent just the actuatable leg joints (q0, q2).
};