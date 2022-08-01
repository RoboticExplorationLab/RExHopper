#pragma once

#include "hopper_mpc/model.h"

// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>

#define ERROR_SIZE 1000

class GazeboBridge {                                                                // The class
 public:                                                                            // Access specifier
  GazeboBridge(Model model, float dt, float g, float mu, bool fixed, bool record);  // constructor
  Model model;
  float dt;
  float g;
  float mu;
  bool fixed;
  bool record;
  void Init(Eigen::Vector4d init_q);
  void SimRun(Eigen::Matrix<double, 5, 1> u);
  void End();

 private:
};