#include "hopper_mpc/sub_mocap.h"

MocapSub::MocapSub(double dt_) {
  // motion cap subscriber/listener
  dt = dt_;

  int N_lookback = 6;
  px_hist.resize(N_lookback);
  py_hist.resize(N_lookback);
  pz_hist.resize(N_lookback);
  t_hist.resize(N_lookback);
  t_mocap = 0;
  // initialize values
  for (int i = 0; i < N_lookback; i++) {
    px_hist.at(i) = 0;
    py_hist.at(i) = 0;
    pz_hist.at(i) = 0;
  }

  sub_mocap = nh_mocap.subscribe("/mocap_node/Robot_1/pose", 1, &MocapSub::MocapCallback, this);
}

// void MocapSub::Init() {}

void MocapSub::MocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_mocap) {
  t_mocap = msg_mocap->header.stamp.toSec();
  Q_raw.x() = msg_mocap->pose.orientation.x;
  Q_raw.y() = msg_mocap->pose.orientation.y;
  Q_raw.z() = msg_mocap->pose.orientation.z;
  Q_raw.w() = msg_mocap->pose.orientation.w;
  p_raw << msg_mocap->pose.position.x, msg_mocap->pose.position.y, msg_mocap->pose.position.z;

  dt_mocap = t_mocap - t_mocap_prev;
  t_mocap_prev = t_mocap;

  // Don't forget: if you make this asynchronous, this next part can't be in the callback function.
  // check if mocap has updated yet
  if (dt_mocap != 0.0) {
    p = p_raw;
    Q = Q_raw;
    // mocap has updated, so it's time to update vector of saved p and t for polyfitting
    std::move(begin(px_hist) + 1, end(px_hist), begin(px_hist));  // shift the vector to the right by one (deleting the first value)
    px_hist.back() = p(0);
    std::move(begin(py_hist) + 1, end(py_hist), begin(py_hist));  // shift the vector to the right by one (deleting the first value)
    py_hist.back() = p(1);
    std::move(begin(pz_hist) + 1, end(pz_hist), begin(pz_hist));  // shift the vector to the right by one (deleting the first value)
    pz_hist.back() = p(2);
    // update vector of saved t
    std::move(begin(t_hist) + 1, end(t_hist), begin(t_hist));  // shift the vector to the right by one (deleting the first value)
    t_hist.back() = t_mocap;
  } else {
    t_hist.back() = *t_hist.end() + dt;  // estimate time since last mocap update
    // if p is not being updated by the mocap, interpolate it using polynomial regression
    p(0) = Utils::PolyFit(t_hist, px_hist, 3, t_hist.back());
    p(1) = Utils::PolyFit(t_hist, py_hist, 3, t_hist.back());
    p(2) = Utils::PolyFit(t_hist, pz_hist, 3, t_hist.back());
  }
}

// void MocapSub::MocapSpin() {
//   ros::spinOnce();
// }