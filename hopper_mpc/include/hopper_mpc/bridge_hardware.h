#pragma once

#include "hopper_can_interface/ODriveCan.h"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/cx5.h"
#include "hopper_mpc/mocap_node.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/wt901.h"
// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"
// boost serialization
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

using namespace hopper::can;

class saved_offsets {
 private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    ar& q0_offset;
    ar& q2_offset;
  }

 public:
  float q0_offset;
  float q2_offset;
  saved_offsets(){};
  saved_offsets(float q0, float q2) : q0_offset(q0), q2_offset(q2) {}
};

class HardwareBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                                // Access specifier
  HardwareBridge(Model model_, double dt_, bool fixed_, bool home_);  // constructor
  // --- virtual function overrides --- //
  void Init() override;
  retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) override;
  void End() override;
  // --- //
  std::string ctrlMode_prev;

 private:
  void Home(std::unique_ptr<ODriveCan>& ODrive, int node_id, int dir);
  void Startup(std::unique_ptr<ODriveCan>& ODrive, int node_id, double tor_lim, double vel_lim);
  void SetPosCtrlMode(std::unique_ptr<ODriveCan>& ODrive, int node_id, double q_init);
  void SetTorCtrlMode(std::unique_ptr<ODriveCan>& ODrive, int node_id);
  std::unique_ptr<ODriveCan> ODriveCANleft;
  std::unique_ptr<ODriveCan> ODriveCANright;
  std::unique_ptr<ODriveCan> ODriveCANyaw;
  double node_id_q0;
  double node_id_q2;
  double node_id_rwr;
  double node_id_rwl;
  double node_id_rwz;
  Eigen::Matrix<double, 5, 1> q_offset;
  Eigen::Matrix<double, 5, 1> GetJointPos();
  Eigen::Matrix<double, 5, 1> GetJointVel();
  Eigen::Matrix<double, 5, 1> GetJointTorqueMeasured();
  Eigen::Vector2d ConvertToODrivePos(Eigen::Vector2d qa);
  Eigen::Vector2d ConvertToODriveVel(Eigen::Vector2d dqa);
  void SetJointPos(Eigen::Vector2d qla_ref);  // only need pos control for leg actuators afaik
  void SetJointTorque(Eigen::Matrix<double, 5, 1> u);
  double TurnsToRadians(double turns);

  std::unique_ptr<MocapNode> mocapPtr;
  std::unique_ptr<Wt901> wt901Ptr;
  std::unique_ptr<Cx5> cx5Ptr;

  Eigen::Vector3d p_prev;

  std::vector<double> px_hist;
  std::vector<double> py_hist;
  std::vector<double> pz_hist;
  std::vector<double> t_hist;
  // double posEst_prev;
  // int count;
  double t_mocap;
  Eigen::Vector2d qa_home;

  std::shared_ptr<saved_offsets> saved;
  std::shared_ptr<saved_offsets> saved_get;
};
