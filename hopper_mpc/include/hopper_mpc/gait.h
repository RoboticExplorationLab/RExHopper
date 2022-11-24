#pragma once

#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"
#include "hopper_mpc/rwa.h"

struct uVals {
  Eigen::Matrix<double, 5, 1> u;
  Eigen::Matrix<double, 2, 1> qla_ref;
  std::string ctrlMode;
};

class Gait {  // The class

 public:  // Access specifier
  Gait(Model model_, double dt_, Eigen::Vector3d peb_ref_, std::shared_ptr<Leg>* legPtr_, std::shared_ptr<Rwa>* rwaPtr_);  // constructor
  void Run();
  uVals Raibert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q, Eigen::Vector3d v, Eigen::Vector3d w,
                Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref, Eigen::Vector3d v_ref, Eigen::Vector3d w_ref);
  uVals KinInvVert(std::string state, std::string state_prev, Eigen::Vector3d p, Eigen::Quaterniond Q, Eigen::Vector3d v, Eigen::Vector3d w,
                   Eigen::Vector3d p_ref, Eigen::Quaterniond Q_ref, Eigen::Vector3d v_ref, Eigen::Vector3d w_ref);
  uVals KinInvStand(Eigen::Quaterniond Q);
  uVals GetUp(Eigen::Quaterniond Q);
  uVals Sit();
  uVals Idle();
  uVals CircleTest();
  Eigen::Vector3d peb_ref;

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
  double x_adj;
  std::shared_ptr<Leg> legPtr;
  std::shared_ptr<Rwa> rwaPtr;
  std::unique_ptr<PID3> pid_vPtr;

  // getup
  Eigen::Vector3d peb_ref_init;
  Eigen::Vector3d peb_ref_final;
  Eigen::VectorXd peb_ref_trajx;
  Eigen::VectorXd peb_ref_trajz;
  int i = 0;
  int N_getup;

  // circletest vars
  double x1 = 0;
  double z1 = -0.4;
  double z = -0.3;  // peb_ref(2);
  double r = 0.1;
  int flip = -1;
};