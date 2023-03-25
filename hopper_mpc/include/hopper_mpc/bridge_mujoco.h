#pragma once
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "hopper_mpc/actuator.h"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"
#include "string"

#define ERROR_SIZE 1000

class MujocoBridge : public Bridge {  // The class
 public:
  using Base = Bridge;                                                                                           // Access specifier
  MujocoBridge(Model model_, double dt_, std::shared_ptr<Leg>* legPtr_, std::string start_, bool skip_homing_);  // constructor
  void Init(double x_adj_) override;
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

  static mjModel* m;      // MuJoCo model
  static mjData* data_m;  // MuJoCo data
  static mjvCamera cam;   // abstract camera
  static mjvOption opt;   // visualization options
  static mjvScene scn;    // abstract scene
  static mjrContext con;  // custom GPU context
                          // mouse interaction
  static bool button_left;
  static bool button_middle;
  static bool button_right;
  static double lastx;
  static double lasty;

  std::unique_ptr<PID1> pid_q0Ptr;
  std::unique_ptr<PID1> pid_q2Ptr;
  std::unique_ptr<Actuator> a0;
  std::unique_ptr<Actuator> a1;
  std::unique_ptr<Actuator> a2;
  std::unique_ptr<Actuator> a3;
  std::unique_ptr<Actuator> a4;

  // static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
  // static void mouse_button(GLFWwindow* window, int button, int act, int mods);
  // static void mouse_move(GLFWwindow* window, double xpos, double ypos);
  // static void scroll(GLFWwindow* window, double xoffset, double yoffset);
  // keyboard callback
  static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
      mj_resetData(m, data_m);
      mj_forward(m, data_m);
    }
  }

  // mouse button callback
  static void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
  }

  // mouse move callback
  static void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
      action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
  }

  // scroll callback
  static void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
  }
};