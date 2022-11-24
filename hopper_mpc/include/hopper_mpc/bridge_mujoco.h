#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/actuator.h"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"
#include "mujoco/glfw3.h"
#include "mujoco/mujoco.h"
#include "string"

#define ERROR_SIZE 1000

class MujocoBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                              // Access specifier
  MujocoBridge(Model model_, double dt_, bool fixed_, bool home_);  // constructor
  void Init() override;
  retVals SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) override;
  void End() override;

 private:
  std::string str;
  GLFWwindow* window;
  mjtNum timezero;
  double_t update_rate;
  mjrRect viewport;
  double t_refresh;
  double refresh_rate;
  std::unique_ptr<PID1> pid_q0Ptr;
  std::unique_ptr<PID1> pid_q2Ptr;
  std::unique_ptr<Actuator> a0;
  std::unique_ptr<Actuator> a1;
  std::unique_ptr<Actuator> a2;
  std::unique_ptr<Actuator> a3;
  std::unique_ptr<Actuator> a4;
};