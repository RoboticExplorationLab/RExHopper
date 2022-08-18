#pragma once

#include "hopper_can_interface/ODriveCan.h"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
// std
#include "Eigen/Core"
#include "Eigen/Dense"
#include "string"

class HardwareBridge {  // The class
 public:
  using Base = Bridge;                                                 // Access specifier
  HardwareBridge(Model model_, float dt_, bool fixed_, bool record_);  // constructor
  // --- virtual function overrides --- //
  void Init() override;
  retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) override;
  void End() override;
  // --- //
  void Calibrate();
  void Subscribe(ODriveCan ODrive, int node_id);
  void Home(ODriveCan ODrive, int node_id, int dir);
  void SetPosCtrl(ODriveCan ODrive, int node_id, double q_init);
  Eigen::Matrix<double, 5, 1> GetJointPosition();
  double TurnsToRadians(double turns);
  std::string ctrlMode_prev;

 private:
  ODriveCan ODriveCAN1;  // leg links
  ODriveCan ODriveCAN2;
  ODriveCan ODriveCAN3;
  double node_id_q0;
  double node_id_q2;
  double node_id_rwr;
  double node_id_rwl;
  double node_id_rwz;
  Eigen::Matrix<double, 5, 1> q_offset_;

  // double posEst_prev;
  // int count;
};