#pragma once

#include "hopper_can_interface/ODriveCan.h"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/sub_cx5.h"
#include "hopper_mpc/sub_mocap.h"
#include "hopper_mpc/wt901.h"
// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"
// boost serialization
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

using namespace hopper::can;

class saved_offsets {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar& q0_offset;
    ar& q2_offset;
  }

 public:
  float q0_offset;
  float q2_offset;
  saved_offsets(){};
  saved_offsets(float q0, float q2) : q0_offset(q0), q2_offset(q2) {}
};

class HardwareBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                                                                             // Access specifier
  HardwareBridge(Model model_, double dt_, std::shared_ptr<Leg>* legPtr_, std::string start_, bool skip_homing_);  // constructor
  // --- virtual function overrides --- //
  void Init(double x_adj_) override;
  retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) override;
  void End() override;
  // --- //
  std::string ctrlMode_prev;

 private:
  void Home(std::unique_ptr<ODriveCan>& ODrive, int node_id, int dir, float cur_lim, float vel_lim);
  void SetPosOffset(std::unique_ptr<ODriveCan>& ODrive, int node_id);
  void Startup(std::unique_ptr<ODriveCan>& ODrive, int node_id, float cur_lim, float vel_lim);
  void SetPosCtrlMode(std::unique_ptr<ODriveCan>& ODrive, int node_id, double q_init);
  void SetTorCtrlMode(std::unique_ptr<ODriveCan>& ODrive, int node_id);
  std::unique_ptr<ODriveCan> ODriveCANleft;
  std::unique_ptr<ODriveCan> ODriveCANright;
  std::unique_ptr<ODriveCan> ODriveCANyaw;
  double node_id_q0;
  double node_id_q2;
  double node_id_rwr;
  double node_id_rwl;
  double node_id_rwz;
  Eigen::Matrix<double, 5, 1> q_offset;
  Eigen::Matrix<double, 5, 1> GetJointPos();
  Eigen::Matrix<double, 5, 1> GetJointVel();
  Eigen::Matrix<double, 5, 1> GetJointTorqueMeasured();
  Eigen::Vector2d ConvertToODrivePos(Eigen::Vector2d qa);
  Eigen::Vector2d ConvertToODriveVel(Eigen::Vector2d dqa);
  void SetJointPos(Eigen::Vector2d qla_ref);  // only need pos control for leg actuators afaik
  void SetJointTorque(Eigen::Matrix<double, 5, 1> u);

  void CheckEndStops(Eigen::Matrix<double, 5, 1> qa);
  void CheckRotorSpeed(Eigen::Matrix<double, 5, 1> dqa);
  // double TurnsToRadians(double turns);

  std::unique_ptr<MocapSub> mocapPtr;
  std::unique_ptr<Wt901> wt901Ptr;
  std::unique_ptr<Cx5> cx5Ptr;

  Eigen::Vector3d p_prev;

  // double posEst_prev;
  // int count;
  double t_mocap;
  Eigen::Vector2d qla_home;

  std::shared_ptr<saved_offsets> saved;
  std::shared_ptr<saved_offsets> saved_get;

  float vel_lim_rmdx10 = 10;
  float cur_lim_rmdx10 = 60;  // 60;

  float vel_lim_r100 = 60;   // max 4400 rpm = 73 rps = 461 rad/s
  float cur_lim_r100 = 104;  // max 104

  float vel_lim_r80 = 60;  // max 5250 rpm = 87.5 rps = 550 rad/s
  float cur_lim_r80 = 46;  // max 46
};
