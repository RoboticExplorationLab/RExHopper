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
  using Base = Bridge;                                                                // Access specifier
  HardwareBridge(Model model, float dt, float g, float mu, bool fixed, bool record);  // constructor
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

 private:
  ODriveCan ODrive_CAN1;  // leg links
  ODriveCan ODrive_CAN2;
  ODriveCan ODrive_CAN3;
  double node_id_q0;
  double node_id_q2;
  double node_id_rw1;
  double node_id_rw2;
  double node_id_rwz;
  Eigen::Matrix<double, 5, 1> a_cal_;
};