#pragma once

#include "hopper_can_interface/ODriveCan.h"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"

using namespace hopper::can;

class HardwareBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                                  // Access specifier
  HardwareBridge(Model model_, double dt_, bool fixed_, bool record_);  // constructor
  // --- virtual function overrides --- //
  void Init() override;
  retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) override;
  void End() override;
  // --- //
  void Calibrate();
  void Subscribe(std::unique_ptr<ODriveCan>& ODrive, int node_id);
  void Home(std::unique_ptr<ODriveCan>& ODrive, int node_id, int dir);
  void SetPosCtrl(std::unique_ptr<ODriveCan>& ODrive, int node_id, double q_init);
  void SetTorCtrl(std::unique_ptr<ODriveCan>& ODrive, int node_id);
  Eigen::Matrix<double, 5, 1> GetJointPosition();
  double TurnsToRadians(double turns);
  std::string ctrlMode_prev;

 private:
  std::unique_ptr<ODriveCan> ODriveCANleft;
  std::unique_ptr<ODriveCan> ODriveCANright;
  std::unique_ptr<ODriveCan> ODriveCANyaw;
  double node_id_q0;
  double node_id_q2;
  double node_id_rwr;
  double node_id_rwl;
  double node_id_rwz;
  Eigen::Matrix<double, 5, 1> q_offset_;
  Eigen::Matrix<double, 5, 1> GetJointPos();
  Eigen::Matrix<double, 5, 1> GetJointVel();
  Eigen::Matrix<double, 5, 1> GetJointPosRaw();

  // double posEst_prev;
  // int count;
};