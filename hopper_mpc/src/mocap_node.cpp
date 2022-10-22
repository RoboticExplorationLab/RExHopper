#include "hopper_mpc/mocap_node.h"

MocapNode::MocapNode() {
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "mocap_node");
  // nh;
  sub_mocap = nh.subscribe("/mocap_node/Robot_2/pose", 1, &MocapNode::MocapCallback, this);
}

// void MocapNode::Init() {}

void MocapNode::MocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_mocap) {
  t_mocap = msg_mocap->header.stamp.toSec();
  Q_mocap.x() = msg_mocap->pose.orientation.x;
  Q_mocap.y() = msg_mocap->pose.orientation.y;
  Q_mocap.z() = msg_mocap->pose.orientation.z;
  Q_mocap.w() = msg_mocap->pose.orientation.w;
  euler_mocap = Utils::QuatToEuler(Q_mocap);
  p_mocap << msg_mocap->pose.position.x, msg_mocap->pose.position.y, msg_mocap->pose.position.z;
  std::cout << "p_mocap = " << p_mocap << "/n";

  dt_mocap = t_mocap - t_mocap_prev;
  t_mocap_prev = t_mocap;
  first_mocap_received = true;
}

void MocapNode::MocapSpin() {
  ros::spinOnce();
}