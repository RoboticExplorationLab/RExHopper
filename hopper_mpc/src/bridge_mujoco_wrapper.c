// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// header file
#include "hopper_mpc/bridge_mujoco_wrapper.h"

#include <stdio.h>
// #include <cstdio>
// #include <cstring>
#include <string.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// MuJoCo data structures
static mjModel* m = NULL;  // MuJoCo model
static mjData* d = NULL;   // MuJoCo data
static mjvCamera cam;      // abstract camera
static mjvOption opt;      // visualization options
static mjvScene scn;       // abstract scene
static mjrContext con;     // custom GPU context

// mouse interaction
static bool button_left = false;
static bool button_middle = false;
static bool button_right = false;
static double lastx = 0;
static double lasty = 0;

static double t_refresh = 0;
static double dt = 0.001;  // shouldn't really be defined here but what choice do I have?

// moved to global space

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
  if (!button_left && !button_middle && !button_right) {
    return;
  }

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
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// init function
GLFWwindow* mujoco_init(char* directory_) {
  // load and compile model
  char error[1000] = "Could not load binary model";
  if (strlen(directory_) > 4 && !strcmp(directory_ + strlen(directory_) - 4, ".mjb")) {
    m = mj_loadModel(directory_, 0);
  } else {
    m = mj_loadXML(directory_, 0, error, 1000);
  }
  if (!m) {
    // mju_error("Load model error: %s", error);
    mju_error(error);
  }

  printf("size of d = %lu\n", sizeof(*d));
  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  return window;
}

void mujoco_set_qpos(double p0, double p1, double p2) {
  d->qpos[0] = p0;
  d->qpos[1] = p1;
  d->qpos[2] = p2;
}

void mujoco_set_qpos_joints(double p7, double p9) {
  d->qpos[7] = p7;  // joint 0
  d->qpos[9] = p9;  // joint 2
}

struct fiveVals mujoco_get_qa() {
  // first seven indices are for base pos and quat in floating body mode
  // for velocities, it's the first six indices
  struct fiveVals a = {d->qpos[7], d->qpos[9], d->qpos[11], d->qpos[12], d->qpos[13]};
  return a;
}

struct fiveVals mujoco_get_dqa() {
  struct fiveVals a = {d->qvel[6], d->qvel[8], d->qvel[10], d->qvel[11], d->qvel[12]};
  return a;
}

struct fiveVals mujoco_get_qa_fixed() {
  struct fiveVals a = {d->qpos[0], d->qpos[2], d->qpos[4], d->qpos[5], d->qpos[6]};
  return a;
}

struct fiveVals mujoco_get_dqa_fixed() {
  struct fiveVals a = {d->qvel[0], d->qvel[2], d->qvel[4], d->qvel[5], d->qvel[6]};
  return a;
}

struct quatVals mujoco_get_quat() {
  struct quatVals a = {d->sensordata[3], d->sensordata[4], d->sensordata[5], d->sensordata[6]};
  return a;
}

struct threeVals mujoco_get_p() {
  struct threeVals a = {d->sensordata[0], d->sensordata[1], d->sensordata[2]};
  return a;
}

struct threeVals mujoco_get_v() {
  struct threeVals a = {d->sensordata[7], d->sensordata[8], d->sensordata[9]};
  return a;
}

struct threeVals mujoco_get_wb() {
  struct threeVals a = {d->sensordata[10], d->sensordata[11], d->sensordata[12]};
  return a;
}

struct fiveVals mujoco_get_tau() {
  struct fiveVals a = {d->sensordata[14], d->sensordata[15], d->sensordata[16], d->sensordata[17], d->sensordata[18]};
  return a;
}

struct fiveVals mujoco_get_rfx() {
  struct fiveVals a = {d->sensordata[19], d->sensordata[22], d->sensordata[25], d->sensordata[28], d->sensordata[31]};
  return a;
}

struct fiveVals mujoco_get_rfy() {
  struct fiveVals a = {d->sensordata[20], d->sensordata[23], d->sensordata[26], d->sensordata[29], d->sensordata[32]};
  return a;
}

struct fiveVals mujoco_get_rfz() {
  struct fiveVals a = {d->sensordata[21], d->sensordata[24], d->sensordata[27], d->sensordata[30], d->sensordata[33]};
  return a;
}

struct threeVals mujoco_get_ab() {
  struct threeVals a = {d->sensordata[34], d->sensordata[35], d->sensordata[36]};
  return a;
}

struct threeVals mujoco_get_aef() {
  struct threeVals a = {d->sensordata[37], d->sensordata[38], d->sensordata[39]};
  return a;
}

struct fiveVals mujoco_get_rtx() {
  struct fiveVals a = {d->sensordata[40], d->sensordata[43], d->sensordata[46], d->sensordata[49], d->sensordata[52]};
  return a;
}

struct fiveVals mujoco_get_rty() {
  struct fiveVals a = {d->sensordata[41], d->sensordata[44], d->sensordata[47], d->sensordata[50], d->sensordata[53]};
  return a;
}

struct fiveVals mujoco_get_rtz() {
  struct fiveVals a = {d->sensordata[42], d->sensordata[45], d->sensordata[48], d->sensordata[51], d->sensordata[54]};
  return a;
}

double mujoco_get_grf() {
  return d->sensordata[13];
}

double mujoco_get_time() {
  return d->time;
}

void mujoco_update(GLFWwindow* window, double tau0, double tau1, double tau2, double tau3, double tau4) {
  // RENDERING STUFF
  double fps = 60;           // 60 frames rendered per real second
  double simhertz = 1 / dt;  // timesteps per simulated second
  // then only render visuals once every 16.66 timesteps
  double refresh_rate = simhertz / fps;

  // step main loop, target real-time simulation and 60 fps rendering
  if (!glfwWindowShouldClose(window)) {
    t_refresh += 1;

    if (t_refresh > refresh_rate) {
      mj_step1(m, d);
      d->ctrl[0] = tau0;  // moves joint 0
      d->ctrl[1] = tau1;  // moves joint 2
      d->ctrl[2] = tau2;  // moves rw0
      d->ctrl[3] = tau3;  // moves rw1
      d->ctrl[4] = tau4;  // moves rwz
      mj_step2(m, d);
      // visualization
      t_refresh = 0;  // reset refresh counter
      // 15 ms is a little smaller than 60 Hz.
      // get framebuffer viewport
      mjrRect viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

      // update scene and render
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);

      // swap OpenGL buffers (blocking call due to v-sync)
      glfwSwapBuffers(window);

      // process pending GUI events, call GLFW callbacks
      glfwPollEvents();
    }
  } else {
    mujoco_end_sim();
  }
}

void mujoco_end_sim() {
  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}