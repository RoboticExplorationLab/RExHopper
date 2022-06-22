#pragma once
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
  void Init();
  void SimRun();
  void End();

 private:
  // std::string error_;

  mjModel* m_;
  mjData* d_;
  mjvCamera cam_;   // abstract camera
  mjvOption opt_;   // visualization options
  mjvScene scn_;    // abstract scene
  mjrContext con_;  // custom GPU context
  GLFWwindow* window_;
  mjtNum timezero_;
  double_t update_rate_;
  mjrRect viewport_;
};