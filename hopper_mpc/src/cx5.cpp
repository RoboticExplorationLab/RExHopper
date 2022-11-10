#include "hopper_mpc/cx5.h"
#include <string>

Cx5::Cx5() {
  // initialize values
  Q.setIdentity();
  omega.setZero();
  alpha.setZero();

  sub_cx5 = nh_cx5.subscribe<sensor_msgs::Imu>(("/nav/filtered_imu/data"), 3, &Cx5::ImuDataCallback, this);
}

void Cx5::ImuDataCallback(const sensor_msgs::Imu::ConstPtr& imu) {
  // ROS_INFO("Quaternion Orientation:    [%f, %f, %f, %f]", imu->orientation.x, imu->orientation.y, imu->orientation.z,
  // imu->orientation.w); ROS_INFO("Angular Velocity:          [%f, %f, %f]", imu->angular_velocity.x, imu->angular_velocity.y,
  // imu->angular_velocity.z); ROS_INFO("Linear Acceleration:       [%f, %f, %f]", imu->linear_acceleration.x, imu->linear_acceleration.y,
  // imu->linear_acceleration.z);
  Q.coeffs() << imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w;
  omega << imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z;
  alpha << imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z;
}

// void Cx5::ImuDataCallback(const sensor_msgs::Imu imu) {
//   Q.coeffs() << imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w;
//   std::cout << Q.coeffs().transpose() << "\n";
// }