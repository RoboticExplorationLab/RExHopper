#include "hopper_mpc/cx5.h"
#include <string>

Cx5::Cx5() {
  // initialize values
  Q.setIdentity();
  omega.setZero();
  alpha.setZero();

  // get the device name parameter
  std::string deviceName;
  ros::NodeHandle params("~");
  params.param<std::string>("device", deviceName, "cx5");
  ROS_INFO("Got device param: %s", deviceName.c_str());

  // clear param for future use
  params.deleteParam("device");

  sub_cx5 = n.subscribe(("/" + deviceName + "/imu/data"), 3, &Cx5::ImuDataCallback, this);
}

void Cx5::ImuDataCallback(const sensor_msgs::Imu::ConstPtr& imu) {
  ROS_INFO("Quaternion Orientation:    [%f, %f, %f, %f]", imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
  ROS_INFO("Angular Velocity:          [%f, %f, %f]", imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
  ROS_INFO("Linear Acceleration:       [%f, %f, %f]", imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

  Q.coeffs() << imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w;
  omega << imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z;
  alpha << imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z;
}