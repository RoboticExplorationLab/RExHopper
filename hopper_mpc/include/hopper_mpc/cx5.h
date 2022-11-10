#pragma once
#include "Eigen/Dense"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class Cx5 {               // The class
 public:                  // Access specifier
  Cx5();                  // constructor
  Eigen::Quaterniond Q;   // quaternion orientation
  Eigen::Vector3d omega;  // angular acc
  Eigen::Vector3d alpha;  // linear acc

 private:
  void ImuDataCallback(const sensor_msgs::Imu::ConstPtr& imu);
  // void ImuDataCallback(const sensor_msgs::Imu imu);
  ros::NodeHandle nh_cx5;
  ros::Subscriber sub_cx5;
};
