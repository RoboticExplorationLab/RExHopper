#include "hopper_mpc/cx5.h"
#include <string>
#include "hopper_mpc/utils.hpp"

Cx5::Cx5() {
  // initialize values
  Q.setIdentity();
  omega.setZero();
  alpha.setZero();

  sub_cx5 = nh_cx5.subscribe<sensor_msgs::Imu>(("/nav/filtered_imu/data"), 3, &Cx5::ImuDataCallback, this);

  Eigen::Matrix3d R1 = Utils::EulerToQuat(-135 * M_PI / 180, 0.0, 0.0).matrix();
  Eigen::Matrix3d R2 = Utils::EulerToQuat(0, 0, 0.5 * M_PI).matrix();
  R = R1 * R2;  // Mounting rotation
  T << -5.0, 0.0, 128.6;
}

void Cx5::ImuDataCallback(const sensor_msgs::Imu::ConstPtr& imu) {
  // ROS_INFO("Quaternion Orientation:    [%f, %f, %f, %f]", imu->orientation.x, imu->orientation.y, imu->orientation.z,
  // imu->orientation.w); ROS_INFO("Angular Velocity:          [%f, %f, %f]", imu->angular_velocity.x, imu->angular_velocity.y,
  // imu->angular_velocity.z); ROS_INFO("Linear Acceleration:       [%f, %f, %f]", imu->linear_acceleration.x, imu->linear_acceleration.y,
  // imu->linear_acceleration.z);
  Q_raw.coeffs() << imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w;
  omega_raw << imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z;
  alpha_raw << imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z;

  Q = R * Q_raw;  // Do this in SensorConnect for faster code... if you trust it...
  omega = R * omega_raw;
  alpha = R * alpha_raw - Utils::Skew(omega) * Utils::Skew(omega) * T;
  // alpha = R * alpha_raw - Q.matrix().transpose() * Eigen::Vector3d(0, 0, 9.81) - Utils::Skew(omega) * Utils::Skew(omega) * T;
}