#include "hopper_mpc/mocap_node.h"

MocapNode::MocapNode() {}

void MocapNode::Init() {
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "mocap_node");
  // nh;
  sub_mocap = nh.subscribe("/mocap_node/Robot_2/pose", 1, &MocapNode::MocapCallback, this);
}

void MocapNode::MocapCallback(const geometry_msgs::PoseStamped::ConstPtr& opti_msg) {
  double opti_t = opti_msg->header.stamp.toSec();
  Eigen::Matrix<double, 3, 1> opti_pos;
  Eigen::Vector3d opti_euler;
  Eigen::Quaterniond opti_quat(opti_msg->pose.orientation.w, opti_msg->pose.orientation.x, opti_msg->pose.orientation.y,
                               opti_msg->pose.orientation.z);
  // opti_euler = Utils::QuatToEuler(opti_quat);
  opti_pos << opti_msg->pose.position.x, opti_msg->pose.position.y, opti_msg->pose.position.z;
  std::cout << opti_pos << "/n";  // TODO: Get this working

  // if (!mocap_ekf_.is_inited() && !ekf_data_.opti_vel_ready()) {
  //   // if ekf is NOT ready and vel data is NOT ready
  //   opti_dt_ = opti_t - opti_t_prev_;
  //   // input dt, pos, and euler into ekf
  //   ekf_data_.input_opti_dt(opti_dt_);
  //   ekf_data_.input_opti_pos(opti_pos);
  //   ekf_data_.input_opti_euler(opti_euler);
  // } else {
  //   opti_dt_ = opti_t - opti_t_prev_;
  //   // input dt, pos, and euler into ekf
  //   ekf_data_.input_opti_dt(opti_dt_);
  //   ekf_data_.input_opti_pos(opti_pos);
  //   ekf_data_.input_opti_euler(opti_euler);
  //   // input vel from optitrack
  //   a1_ctrl_states.vel_mocap_ekf = ekf_data_.opti_vel;
  //   // update filter
  //   mocap_ekf_.update_filter_with_opti(ekf_data_);
  // }
  // if (!mocap_ekf_.is_inited() && ekf_data_.opti_vel_ready()) {
  //   // if ekf is not ready and vel data is ready
  //   mocap_ekf_.init_filter(ekf_data_, ekf_data_.opti_pos);
  //   // get the current state vector x of the ekf
  //   // Eigen::Matrix<double, EKF_STATE_SIZE, 1> kf_state = mocap_ekf_.get_state();
  //   Eigen::Matrix<double, EKF_STATE_SIZE, 1> kf_state = ekf->x;  // might be better
  //   // just... feeding pos from one object to another...
  //   a1_ctrl_states.root_pos = kf_state.segment<3>(0);
  //   a1_ctrl_states.root_pos_d.segment<2>(0) = kf_state.segment<2>(0);
  // }
  // opti_t_prev_ = opti_t;
  // first_mocap_received = true;
}

void MocapNode::MocapSpin() {
  ros::spinOnce();
}