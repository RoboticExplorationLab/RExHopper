#include "hopper_mpc/bridge_mujoco.h"
#include "mujoco/glfw3.h"
#include "mujoco/mujoco.h"
// for sleep timers

#include <chrono>
#include <iostream>
#include <thread>
// for getcwd()
#include <unistd.h>
mjModel* m;      // MuJoCo model
mjData* d;       // MuJoCo data
mjvCamera cam;   // abstract camera
mjvOption opt;   // visualization options
mjvScene scn;    // abstract scene
mjrContext con;  // custom GPU context
                 // mouse interaction
bool button_left;
bool button_middle;
bool button_right;
double lastx;
double lasty;

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

MujocoBridge::MujocoBridge(Model model_, double dt_, bool fixed_, bool skip_homing_) : Base(model_, dt_, fixed_, skip_homing_) {}

void MujocoBridge::Init() {
  char error[ERROR_SIZE] = "Could not load binary model";

  std::string path_mjcf;
  if (fixed == true) {
    path_mjcf = model.mjcf_fixed_path;
  } else {
    path_mjcf = model.mjcf_path;
  }
  char* cwd;
  char buff[PATH_MAX + 1];
  cwd = getcwd(buff, PATH_MAX + 1);  // get current working directory
  std::string path_cwd = cwd;
  str = path_cwd + path_mjcf;  // combine current directory with relative path for mjcf

  char* dir = new char[PATH_MAX + 1];  // just needs to be larger than the actual string
  strcpy(dir, str.c_str());            // converting back to char... probably wasteful but whatevs
  if (std::strlen(dir) > 4 && !std::strcmp(dir + std::strlen(dir) - 4, ".mjb")) {
    m = mj_loadModel(dir, 0);
  } else {
    m = mj_loadXML(dir, 0, error, 1000);
  }
  if (!m) {
    mju_error_s("Load model error: %s", error);
  }
  // m = mj_loadXML(mjcf, 0, error, ERROR_SIZE);  // MuJoCo model
  d = mj_makeData(m);  // MuJoCo data

  button_left = false;
  button_middle = false;
  button_right = false;
  lastx = 0;
  lasty = 0;
  // init GLFW
  if (!glfwInit()) mju_error("Could not initialize GLFW");

  // create window, make OpenGL context current, request v-sync
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scn, 2000);               // space for 2000 objects
  mjr_makeContext(m, &con, mjFONTSCALE_150);  // model-specific context

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // run main loop, target real-time simulation and 60 fps rendering
  timezero = d->time;
  update_rate = dt;  // update rate is same as timestep size for now

  // making sure the first time step updates the ctrl previous_time
  // last_update = timezero - 1.0 / ctrl_update_freq;

  // RENDERING STUFF
  double fps = 60;           // 60 frames rendered per real second
  double simhertz = 1 / dt;  // timesteps per simulated second
  // then only render visuals once every 16.66 timesteps
  refresh_rate = simhertz / fps;
  t_refresh += 0;

  // initialize variables

  // actuator models
  ActuatorModel r80;
  r80.name = "r80";
  r80.v_max = 48;
  r80.kt = 0.0868;
  r80.omega_max = 4600 * (2 * M_PI / 60);
  r80.tau_max = 4;
  r80.r = 0.125;
  r80.i_max = 46;
  r80.gr = 1;

  ActuatorModel rmdx10;
  rmdx10.name = "rmdx10";
  rmdx10.v_max = 48;
  rmdx10.kt = 1.73 / 7;
  rmdx10.omega_max = 250 * 7 * (2 * M_PI / 60);
  rmdx10.tau_max = 50 / 7;
  rmdx10.r = 0.3;
  rmdx10.i_max = 30;
  rmdx10.gr = 7;

  ActuatorModel r100;
  r100.name = "r100";
  r100.v_max = 48;
  r100.kt = 0.106;
  r100.omega_max = 3800 * (2 * M_PI / 60);
  r100.tau_max = 11.24;
  r100.r = 0.051;
  r100.i_max = 104;
  r100.gr = 1;

  a0.reset(new Actuator(rmdx10, dt));
  a1.reset(new Actuator(rmdx10, dt));
  a2.reset(new Actuator(r100, dt));
  a3.reset(new Actuator(r100, dt));
  a4.reset(new Actuator(r80, dt));

  qa_cal << model.q_init(0), model.q_init(2), 0, 0, 0;
  double kp = model.k_kin(0);
  double kd = model.k_kin(1);
  pid_q0Ptr.reset(new PID1(dt, kp, 0.0, kd));
  pid_q2Ptr.reset(new PID1(dt, kp, 0.0, kd));
  sh = 0;

  if (skip_homing == true) {  // the robot is not homing, so it must start from a sitting position
    d->qpos[0] = model.p0_sit(0);
    d->qpos[1] = model.p0_sit(1);
    d->qpos[2] = model.p0_sit(2);
    d->qpos[7] = model.qla_sit(0) - qa_cal(0);  // joint 0
    d->qpos[9] = model.qla_sit(1) - qa_cal(1);  // joint 2
  } else {
    d->qpos[0] = model.p0(0);
    d->qpos[1] = model.p0(1);
    d->qpos[2] = model.p0(2);
  }
}

retVals MujocoBridge::SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) {
  if (!glfwWindowShouldClose(window)) {
    // if leg is using direct position controller, replace leg torque control values with qa_ref based pid torques
    if (ctrlMode == "Pos") {  // this is the simulated equivalent of using ODrive's position controller
      u(0) = pid_q0Ptr->PIDControl(qa(0), qla_ref(0));
      u(1) = pid_q2Ptr->PIDControl(qa(1), qla_ref(1));
    }
    // get dqa
    if (fixed == true) {
      dqa << d->qvel[0], d->qvel[2], d->qvel[4], d->qvel[5], d->qvel[6];
    } else {
      dqa << d->qvel[6], d->qvel[8], d->qvel[10], d->qvel[11], d->qvel[12];
    }
    u *= -1;
    mj_step1(m, d);  // mj_step(m, d);
    auto [tau0, i0, v0] = a0->Actuate(u(0), dqa(0));
    auto [tau1, i1, v1] = a1->Actuate(u(1), dqa(1));
    auto [tau2, i2, v2] = a2->Actuate(u(2), dqa(2));
    auto [tau3, i3, v3] = a3->Actuate(u(3), dqa(3));
    auto [tau4, i4, v4] = a4->Actuate(u(4), dqa(4));
    d->ctrl[0] = tau0;  // moves joint 0
    d->ctrl[1] = tau1;  // moves joint 2
    d->ctrl[2] = tau2;  // moves rw0
    d->ctrl[3] = tau3;  // moves rw1
    d->ctrl[4] = tau4;  // moves rwz
    mj_step2(m, d);

    // std::cout << tau0 << ", " << tau1 << ", " << tau2 << ", " << tau3 << ", " << tau4 << "\n";

    // get measurements
    if (fixed == true) {
      qa_raw << d->qpos[0], d->qpos[2], d->qpos[4], d->qpos[5], d->qpos[6];
      dqa << d->qvel[0], d->qvel[2], d->qvel[4], d->qvel[5], d->qvel[6];
    } else {
      qa_raw << d->qpos[7], d->qpos[9], d->qpos[11], d->qpos[12], d->qpos[13];
      dqa << d->qvel[6], d->qvel[8], d->qvel[10], d->qvel[11], d->qvel[12];
      // first seven indices are for base pos and quat in floating body mode
      // for velocities, it's the first six indices
    }

    qa = qa_raw + qa_cal;  // Correct the angle. Make sure this only happens once per time step
    // TODO: Should sensor stuff go after the integrate?
    p << d->sensordata[0], d->sensordata[1], d->sensordata[2];

    if (d->time == 0.001) {
      Q.setIdentity();  // sensordata quaternion initializes as all zeros which is invalid
    } else {
      Q.w() = d->sensordata[3];
      Q.x() = d->sensordata[4];
      Q.y() = d->sensordata[5];
      Q.z() = d->sensordata[6];
    }

    v << d->sensordata[7], d->sensordata[8], d->sensordata[9];

    wb << d->sensordata[10], d->sensordata[11], d->sensordata[12];

    grf_normal = d->sensordata[13];
    sh = grf_normal >= 60 ? true : false;  // check if contact is legit

    tau << d->sensordata[14], d->sensordata[15], d->sensordata[16], d->sensordata[17], d->sensordata[18];
    tau_ref = u;

    rf_x << d->sensordata[19], d->sensordata[22], d->sensordata[25], d->sensordata[28], d->sensordata[31];
    rf_y << d->sensordata[20], d->sensordata[23], d->sensordata[26], d->sensordata[29], d->sensordata[32];
    rf_z << d->sensordata[21], d->sensordata[24], d->sensordata[27], d->sensordata[30], d->sensordata[33];

    ab << d->sensordata[34], d->sensordata[35], d->sensordata[36];   // base accelerometer
    aef << d->sensordata[37], d->sensordata[38], d->sensordata[39];  // foot accelerometer

    t_refresh += 1;
    if (t_refresh > refresh_rate) {
      // visualization
      t_refresh = 0;  // reset refresh counter
      // 15 ms is a little smaller than 60 Hz.
      // get framebuffer viewport
      viewport = {0, 0, 0, 0};
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
    End();
  }

  return retVals{p, Q, v, wb, ab, aef, qa, dqa};
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