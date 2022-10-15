#include "Eigen/Core"
#include "Eigen/Dense"
// ros
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "hopper_mpc/utils.hpp"

Eigen::Vector3d QuatToEuler(Eigen::Quaterniond quat) {
  Eigen::Vector3d rst;

  // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
  Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
  double x = coeff(0);
  double y = coeff(1);
  double z = coeff(2);
  double w = coeff(3);

  double y_sqr = y * y;

  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + y_sqr);

  rst[0] = atan2(t0, t1);

  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > +1.0 ? +1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  rst[1] = asin(t2);

  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (y_sqr + z * z);
  rst[2] = atan2(t3, t4);
  return rst;
};

void MocapCallback(const geometry_msgs::PoseStamped::ConstPtr& opti_msg) {
  double opti_t = opti_msg->header.stamp.toSec();
  Eigen::Matrix<double, 3, 1> opti_pos;
  Eigen::Vector3d opti_euler;
  Eigen::Quaterniond opti_quat(opti_msg->pose.orientation.w, opti_msg->pose.orientation.x, opti_msg->pose.orientation.y,
                               opti_msg->pose.orientation.z);

  opti_euler = QuatToEuler(opti_quat);
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

int main(int argc, char** argv) {
  // mocap stuff
  // ros::NodeHandle nh;
  // ros::Subscriber sub_mocap_;
  // geometry_msgs::PoseStamped mocap_msg_;
  // double opti_dt_;
  // double opti_t_prev_;
  // double init_yaw_;
  // bool first_mocap_received = false;
  // Mocap EKF
  // A1SensorData ekf_data_;
  // A1KFCombineLOWithFoot mocap_ekf_;
  // end mocap stuff

  //   int argc = 0;
  //   char **argv = NULL;
  ros::init(argc, argv, "mocap_node");
  // std::mutex ekf_data_mutex;
  // sub mocap data
  ros::NodeHandle nh;
  ros::Subscriber sub_mocap = nh.subscribe("/mocap_node/Robot_2/pose", 1, MocapCallback);
  //   geometry_msgs::PoseStamped mocap_msg = geometry_msgs::PoseStamped();

  ros::spin();

  return 0;
}