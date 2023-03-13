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

// #include <stdio.h>
#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// MuJoCo data structures
mjModel* m = NULL;  // MuJoCo model
mjData* d = NULL;   // MuJoCo data
mjvCamera cam;      // abstract camera
mjvOption opt;      // visualization options
mjvScene scn;       // abstract scene
mjrContext con;     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// moved to global space
// create window, make OpenGL context current, request v-sync
GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);

double t_refresh = 0;
double dt = 0.001;  // shouldn't really be defined here but what choice do I have?

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
void mujoco_init(char* directory_) {
  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(directory_) > 4 && !std::strcmp(directory_ + std::strlen(directory_) - 4, ".mjb")) {
    m = mj_loadModel(directory_, 0);
  } else {
    m = mj_loadXML(directory_, 0, error, 1000);
  }
  if (!m) {
    // mju_error("Load model error: %s", error);
    mju_error(error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

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

fiveVals mujoco_get_qa() {
  // first seven indices are for base pos and quat in floating body mode
  // for velocities, it's the first six indices
  return {d->qpos[7], d->qpos[9], d->qpos[11], d->qpos[12], d->qpos[13]};
}

fiveVals mujoco_get_dqa() {
  return {d->qvel[6], d->qvel[8], d->qvel[10], d->qvel[11], d->qvel[12]};
}

fiveVals mujoco_get_qa_fixed() {
  return {d->qpos[0], d->qpos[2], d->qpos[4], d->qpos[5], d->qpos[6]};
}

fiveVals mujoco_get_dqa_fixed() {
  return {d->qvel[0], d->qvel[2], d->qvel[4], d->qvel[5], d->qvel[6]};
}

quatVals mujoco_get_quat() {
  return {d->sensordata[3], d->sensordata[4], d->sensordata[5], d->sensordata[6]};
}

threeVals mujoco_get_p() {
  return {d->sensordata[0], d->sensordata[1], d->sensordata[2]};
}

threeVals mujoco_get_v() {
  return {d->sensordata[7], d->sensordata[8], d->sensordata[9]};
}

threeVals mujoco_get_wb() {
  return {d->sensordata[10], d->sensordata[11], d->sensordata[12]};
}

fiveVals mujoco_get_tau() {
  return {d->sensordata[14], d->sensordata[15], d->sensordata[16], d->sensordata[17], d->sensordata[18]};
}

fiveVals mujoco_get_rfx() {
  return {d->sensordata[19], d->sensordata[22], d->sensordata[25], d->sensordata[28], d->sensordata[31]};
}

fiveVals mujoco_get_rfy() {
  return {d->sensordata[20], d->sensordata[23], d->sensordata[26], d->sensordata[29], d->sensordata[32]};
}

fiveVals mujoco_get_rfz() {
  return {d->sensordata[21], d->sensordata[24], d->sensordata[27], d->sensordata[30], d->sensordata[33]};
}

threeVals mujoco_get_ab() {
  return {d->sensordata[34], d->sensordata[35], d->sensordata[36]};
}

threeVals mujoco_get_aef() {
  return {d->sensordata[37], d->sensordata[38], d->sensordata[39]};
}

fiveVals mujoco_get_rtx() {
  return {d->sensordata[40], d->sensordata[43], d->sensordata[46], d->sensordata[49], d->sensordata[52]};
}

fiveVals mujoco_get_rty() {
  return {d->sensordata[41], d->sensordata[44], d->sensordata[47], d->sensordata[50], d->sensordata[53]};
}

fiveVals mujoco_get_rtz() {
  return {d->sensordata[42], d->sensordata[45], d->sensordata[48], d->sensordata[51], d->sensordata[54]};
}

double mujoco_get_grf() {
  return d->sensordata[13];
}

double mujoco_get_time() {
  return d->time;
}

void mujoco_update(double tau0, double tau1, double tau2, double tau3, double tau4) {
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