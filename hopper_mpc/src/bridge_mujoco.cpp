#include "hopper_mpc/bridge_mujoco.h"
#include "mujoco/glfw3.h"
#include "mujoco/mujoco.h"
// for sleep timers
#include <chrono>
#include <iostream>
#include <thread>

char error[ERROR_SIZE] = "Could not load binary model";  // forced to make this global because I don't know how to code

mjModel* m = mj_loadXML("/workspaces/RosDockerWorkspace/src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08.xml", 0, error,
                        ERROR_SIZE);  // MuJoCo model
mjData* d = mj_makeData(m);           // MuJoCo data
mjvCamera cam;                        // abstract camera
mjvOption opt;                        // visualization options
mjvScene scn;                         // abstract scene
mjrContext con;                       // custom GPU context
                                      // mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
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
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

MujocoBridge::MujocoBridge(Model model, float dt, float g, float mu, bool fixed, bool record) {
  // constructor
  model = model;
  dt = dt;
  g = g;
  mu = mu;
  fixed = fixed;
  record = record;
}

void MujocoBridge::Init(Eigen::Vector4d init_q) {
  // init GLFW
  if (!glfwInit()) mju_error("Could not initialize GLFW");

  // create window, make OpenGL context current, request v-sync
  window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scn, 2000);               // space for 2000 objects
  mjr_makeContext(m, &con, mjFONTSCALE_150);  // model-specific context

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window_, keyboard);
  glfwSetCursorPosCallback(window_, mouse_move);
  glfwSetMouseButtonCallback(window_, mouse_button);
  glfwSetScrollCallback(window_, scroll);

  // initial position
  // d->qpos[0] = init_q[0];
  // d->qpos[1] = 0;
  // d->qpos[2] = init_q[1];
  // d->qpos[3] = 0;
  // d->qpos[4] = init_q[2];
  // d->qpos[5] = init_q[3];
  // d->qpos[6] = init_q[4];

  // run main loop, target real-time simulation and 60 fps rendering
  timezero_ = d->time;
  update_rate_ = dt;  // update rate is same as timestep size for now

  // making sure the first time step updates the ctrl previous_time
  // last_update = timezero - 1.0 / ctrl_update_freq;
}

void MujocoBridge::SimRun(Eigen::Matrix<double, 5, 1> u) {
  // use the first while condition if you want to simulate for a period.
  //    while( !glfwWindowShouldClose(window) and d->time-timezero < 1.5)
  if (!glfwWindowShouldClose(window_)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;

    while (d->time - simstart < 1.0 / 60.0) {
      mj_step(m, d);
      d->ctrl[0] = u[0];
      d->ctrl[1] = 0;
      d->ctrl[2] = u[1];
      d->ctrl[3] = 0;
      d->ctrl[4] = u[2];
      d->ctrl[5] = u[3];
      d->ctrl[6] = u[4];
      mj_step(m, d);
      // d->ctrl[7] = 1;
    }

    std::cout << "actuator force: " << (d->actuator_force[0]) << ", " << (d->actuator_force[1]) << ", " << (d->actuator_force[2]) << ", "
              << (d->actuator_force[3]) << ", " << (d->actuator_force[4]) << ", " << (d->actuator_force[5]) << ", "
              << (d->actuator_force[6]) << std::endl;
    // std::cout << "qpos: " << (d->qpos[0]) << ", " << (d->qpos[1]) << ", " << (d->qpos[2]) << ", " << (d->qpos[3]) << ", " << (d->qpos[4])
    //           << ", " << (d->qpos[5]) << ", " << (d->qpos[6]) << std::endl;
    std::cout << "ctrl: " << (d->ctrl[0]) << ", " << (d->ctrl[1]) << ", " << (d->ctrl[2]) << ", " << (d->ctrl[3]) << ", " << (d->ctrl[4])
              << ", " << (d->ctrl[5]) << ", " << (d->ctrl[6]) << std::endl;
    // 15 ms is a little smaller than 60 Hz.
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    // get framebuffer viewport
    viewport_ = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport_, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  } else {
  }
}

void MujocoBridge::End() {
  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);
  mj_deactivate();

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
  ;
}