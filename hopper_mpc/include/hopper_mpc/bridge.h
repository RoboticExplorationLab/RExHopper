#pragma once

#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"

struct retVals {  // Declare a local structure
  Eigen::Vector3d p;
  Eigen::Quaterniond Q;
  Eigen::Vector3d v;                // linear vel in WORLD frame
  Eigen::Vector3d wb;               // angular vel in BODY frame
  Eigen::Vector3d ab;               // linear acceleration in BODY frame
  Eigen::Vector3d aef;              // linear foot acceleration in FOOT frame
  Eigen::Matrix<double, 5, 1> qa;   // actuated joint positions
  Eigen::Matrix<double, 5, 1> dqa;  // actuated joint velocities
};

class Bridge {                                                                                             // The class
 public:                                                                                                   // Access specifier
  Bridge(Model model_, double dt_, std::shared_ptr<Leg>* legPtr_, std::string start_, bool skip_homing_);  // constructor
  virtual void Init(double x_adj_) = 0;
  virtual retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) = 0;
  virtual void End() = 0;
  Eigen::Matrix<double, 5, 1> tau;      // measured torque
  Eigen::Matrix<double, 5, 1> tau_ref;  // ref torque before gear ratios
  double grf_normal;
  Eigen::Matrix<double, 5, 1> rf_x;  // measured force on joints
  Eigen::Matrix<double, 5, 1> rf_y;  // measured force on joints
  Eigen::Matrix<double, 5, 1> rf_z;  // measured force on joints
  bool sh;
  bool stop = false;

 protected:
  Model model;
  double dt;
  double g;
  double mu;
  std::string start;
  bool skip_homing;
  Eigen::Matrix<double, 5, 1> qa_cal;
  Eigen::Matrix<double, 5, 1> qa_raw;

  Eigen::Vector3d p;
  Eigen::Quaterniond Q;
  Eigen::Vector3d v;    // linear vel in WORLD frame
  Eigen::Vector3d wb;   // angular vel in BODY frame
  Eigen::Vector3d ab;   // linear acc in BODY frame
  Eigen::Vector3d aef;  // linear foot acc in FOOT frame

  Eigen::Matrix<double, 5, 1> qa;
  Eigen::Matrix<double, 5, 1> dqa;

  std::shared_ptr<Leg> legPtr;
};
