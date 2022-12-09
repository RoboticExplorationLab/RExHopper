#pragma once
#include "Eigen/Dense"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class Cx5 {               // The class
 public:                  // Access specifier
  Cx5();                  // constructor
  Eigen::Quaterniond Q;   // quaternion orientation
  Eigen::Vector3d omega;  // angular vel at CoM in base frame
  Eigen::Vector3d alpha;  // linear acc at CoM in base frame

 private:
  void ImuDataCallback(const sensor_msgs::Imu::ConstPtr& imu);
  // void ImuDataCallback(const sensor_msgs::Imu imu);
  ros::NodeHandle nh_cx5;
  ros::Subscriber sub_cx5;

  Eigen::Quaterniond Q_raw;
  Eigen::Vector3d alpha_raw;  // linear acc raw data
  Eigen::Vector3d omega_raw;  // angular vel raw data
  Eigen::Vector3d T;          // translation in body frame from CoM to IMU

  Eigen::Matrix3d R;
  // Eigen::Matrix3d R1;
  // Eigen::Matrix3d R2;
};
