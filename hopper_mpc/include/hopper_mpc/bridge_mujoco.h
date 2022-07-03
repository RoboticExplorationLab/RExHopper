#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/model.h"
#include "mujoco/glfw3.h"
#include "mujoco/mujoco.h"
#include "string"

#define ERROR_SIZE 1000

class MujocoBridge {                                                                // The class
 public:                                                                            // Access specifier
  MujocoBridge(Model model, float dt, float g, float mu, bool fixed, bool record);  // constructor
  Model model;
  float dt;
  float g;
  float mu;
  bool fixed;
  bool record;
  void Init(Eigen::Vector4d init_q);
  void SimRun(Eigen::Matrix<double, 5, 1> u);
  void End();

 private:
  GLFWwindow* window_;
  mjtNum timezero_;
  double_t update_rate_;
  mjrRect viewport_;
};