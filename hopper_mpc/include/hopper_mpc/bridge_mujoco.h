#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
#include "mujoco/glfw3.h"
#include "mujoco/mujoco.h"
#include "string"

#define ERROR_SIZE 1000

class MujocoBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                                                 // Access specifier
  MujocoBridge(Model model, double dt, double g, double mu, bool fixed, bool record);  // constructor
  void Init() override;
  void SimRun(Eigen::Matrix<double, 5, 1> u) override;
  void End() override;

 private:
  GLFWwindow* window_;
  mjtNum timezero_;
  double_t update_rate_;
  mjrRect viewport_;
};