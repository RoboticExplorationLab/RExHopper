#pragma once

#include "hopper_can_interface/ODriveCan.h"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"
// ros
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

using namespace hopper::can;

class HardwareBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                                  // Access specifier
  HardwareBridge(Model model_, double dt_, bool fixed_, bool record_);  // constructor
  // --- virtual function overrides --- //
  void Init() override;
  retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) override;
  void End() override;
  // --- //
  std::string ctrlMode_prev;

 private:
  // mocap stuff
  ros::NodeHandle nh;
  ros::Subscriber sub_mocap_;
  geometry_msgs::PoseStamped mocap_msg_;
  double opti_dt_;
  double opti_t_prev_;
  double init_yaw_;
  bool first_mocap_received = false;
  // Mocap EKF
  // A1SensorData ekf_data_;
  // A1KFCombineLOWithFoot mocap_ekf_;
  // end mocap stuff

  void Home(std::unique_ptr<ODriveCan>& ODrive, int node_id, int dir);
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
  Eigen::Vector2d ConvertToODrivePos(Eigen::Vector2d qa);
  Eigen::Vector2d ConvertToODriveVel(Eigen::Vector2d dqa);
  void SetJointPos(Eigen::Vector2d qla_ref);  // only need pos control for leg actuators afaik
  void SetJointTorque(Eigen::Matrix<double, 5, 1> u);
  double TurnsToRadians(double turns);
  void HardwareBridge::MocapCallback(const geometry_msgs::PoseStamped::ConstPtr& opti_msg)
  // double posEst_prev;
  // int count;
};