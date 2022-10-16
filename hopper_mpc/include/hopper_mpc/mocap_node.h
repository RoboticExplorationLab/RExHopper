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
  void Init();
  void MocapCallback(const geometry_msgs::PoseStamped::ConstPtr& opti_msg);
  void MocapSpin();

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub_mocap;
};