#include "Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "sub_cx5.h"
#include "utils.hpp"

// rosrun quaternion_test quaternion_test_node
// rosrun rviz rviz
// change global options -> Fixed Frame = sensor_wgs84
// Add -> "by topic" -> quat_topic
// Add "by display type"->rviz_imu_plugin/imu -> Topic -> nav/filtered/imu/data
int main(int argc, char** argv) {
  // Initialize and start the node
  ros::init(argc, argv, "quaternion_test");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("quat_topic", 1000);

  Cx5 sub_cx5;
  geometry_msgs::PoseStamped pose_msg;
  // odom.header.stamp = current_time;
  pose_msg.header.frame_id = "sensor_wgs84";
  pose_msg.pose.position.x = 0;
  pose_msg.pose.position.y = 0;
  pose_msg.pose.position.z = 0;
  // Define and create some messages

  ros::Rate(200);
  int k = 0;
  // have to wait for the subscriber to pick up a message
  while (sub_cx5.Q.w() == 1) {
    k += 1;
    ros::spinOnce();
  }
  std::cout << "k = " << k << "\n";
  Eigen::Quaterniond Q0 = sub_cx5.Q;
  // Eigen::Quaterniond Q_offset = Utils::ExtractYawQuat(sub_cx5.Q);  // trusting the accuracy of IMU's down vector
  Eigen::Quaterniond Q_offset = sub_cx5.Q.conjugate();  // assuming perfectly flat starting position
  std::cout << "Q0 = " << Q0.coeffs().transpose() << "\n";
  std::cout << "Q_offset = " << Q_offset.coeffs().transpose() << "\n";

  while (ros::ok) {
    pub.publish(pose_msg);
    Q0 = sub_cx5.Q;
    Eigen::Quaterniond Q = (Utils::GenYawQuat(45 * M_PI / 180) * Q0).normalized();
    // Eigen::Quaterniond Q = (Q_offset * Q0).normalized();  // For initial offset adj
    // Eigen::Quaterniond Q = (Utils::ExtractYawQuat(Q0).conjugate() * Q0).normalized();  // for always facing forward
    pose_msg.pose.orientation.x = Q.x();
    pose_msg.pose.orientation.y = Q.y();
    pose_msg.pose.orientation.z = Q.z();
    pose_msg.pose.orientation.w = Q.w();
    ros::spinOnce();
  }
}
