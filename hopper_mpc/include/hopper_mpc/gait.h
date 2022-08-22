#pragma once

#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"
#include "hopper_mpc/rwa.h"

class Gait {  // The class

 public:                                                                                         // Access specifier
  Gait(Model model_, double dt_, std::shared_ptr<Leg>* legPtr_, std::shared_ptr<Rwa>* rwaPtr_);  // constructor
  void Run();
  Eigen::Matrix<double, 5, 1> uRaibert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q,
                                       Eigen::Vector3d v, Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref,
                                       Eigen::Vector3d v_ref, Eigen::Vector3d w_ref);
  Eigen::Matrix<double, 5, 1> uKinInvVert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q,
                                          Eigen::Vector3d v, Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref,
                                          Eigen::Vector3d v_ref, Eigen::Vector3d w_ref);
  Eigen::Matrix<double, 5, 1> uKinInvStand(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q,
                                           Eigen::Vector3d v, Eigen::Vector3d w, Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref,
                                           Eigen::Vector3d v_ref, Eigen::Vector3d w_ref);

 private:
  Eigen::Matrix<double, 5, 1> u;  // controls
  Eigen::Vector3d pe_ref;         // end effector ref position
  Eigen::Vector3d pf_ref;         // footstep ref position
  Eigen::Quaterniond Q;
  Eigen::Quaterniond Q_z;
  // Eigen::Quaterniond Q_ref;
  double z_ref;
  Eigen::Matrix3d R_z;

  Model model;
  double dt;  // timestep size
  std::shared_ptr<Leg> legPtr;
  std::shared_ptr<Rwa> rwaPtr;
  std::unique_ptr<PID3> pid_vPtr;
};