#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
// ros
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "hopper_mpc/utils.hpp"

class MocapNode {  // The class

 public:        // Access specifier
  MocapNode();  // constructor
  // void Init();
  void MocapCallback(const geometry_msgs::PoseStamped::ConstPtr& opti_msg);
  void MocapSpin();
  bool first_mocap_received = false;
  Eigen::Vector3d p_mocap;
  Eigen::Vector3d euler_mocap;
  Eigen::Quaterniond Q_mocap;
  double dt_mocap;
  double t_mocap;

 private:
  double t_mocap_prev = 0;
  ros::NodeHandle nh;
  ros::Subscriber sub_mocap;
};