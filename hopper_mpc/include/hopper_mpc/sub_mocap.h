#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
// ros
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "hopper_mpc/utils.hpp"

class MocapSub {  // The class

 public:                 // Access specifier
  MocapSub(double dt_);  // constructor
  // void Init();
  void MocapCallback(const geometry_msgs::PoseStamped::ConstPtr& opti_msg);
  Eigen::Vector3d p;
  Eigen::Quaterniond Q;

 private:
  double dt;
  double t_mocap_prev = 0;
  ros::NodeHandle nh_mocap;
  ros::Subscriber sub_mocap;
  double dt_mocap;
  double t_mocap;
  Eigen::Vector3d p_raw;
  Eigen::Quaterniond Q_raw;
  std::vector<double> px_hist;
  std::vector<double> py_hist;
  std::vector<double> pz_hist;
  std::vector<double> t_hist;
};