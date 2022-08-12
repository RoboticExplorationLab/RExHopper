#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"
#include "mujoco/glfw3.h"
#include "mujoco/mujoco.h"
#include "string"

#define ERROR_SIZE 1000

class MujocoBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                                                 // Access specifier
  MujocoBridge(Model model, double dt, double g, double mu, bool fixed, bool record);  // constructor
  void Init() override;
  retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) override;
  void End() override;

 private:
  GLFWwindow* window_;
  mjtNum timezero_;
  double_t update_rate_;
  mjrRect viewport_;
  double t_refresh_;
  double refresh_rate_;
  std::unique_ptr<PID1> pid_q0Ptr_;
  std::unique_ptr<PID1> pid_q2Ptr_;
};