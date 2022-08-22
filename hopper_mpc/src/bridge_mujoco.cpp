#include "hopper_mpc/bridge_mujoco.h"
#include "mujoco/glfw3.h"
#include "mujoco/mujoco.h"
// for sleep timers
#include <chrono>
#include <iostream>
#include <thread>

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

MujocoBridge::MujocoBridge(Model model_, double dt_, bool fixed_, bool record_) : Base(model_, dt_, fixed_, record_) {}

void MujocoBridge::Init() {
  char error[ERROR_SIZE] = "Could not load binary model";
  if (fixed == true) {
    str = "/workspaces/RosDockerWorkspace/src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08_mjcf_fixed.xml";
  } else {
    str = "/workspaces/RosDockerWorkspace/src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08_mjcf.xml";
  }
  char* dir = new char[150];  // just needs to be larger than the actual string
  strcpy(dir, str.c_str());
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

  // initialize variables
  qa_cal << model.q_init(0), model.q_init(2), 0, 0, 0;
  double kp = 45;
  pid_q0Ptr.reset(new PID1(dt, kp, 0.0, kp * 0.02));
  pid_q2Ptr.reset(new PID1(dt, kp, 0.0, kp * 0.02));
  sh = 0;

  // RENDERING STUFF
  double fps = 60;           // 60 frames rendered per real second
  double simhertz = 1 / dt;  // 1000 steps per simulated second
  // then only render visuals once every 16.66 timesteps
  refresh_rate = simhertz / fps;
  t_refresh += 0;
}

retVals MujocoBridge::SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) {
  if (!glfwWindowShouldClose(window)) {
    qa_raw << d->qpos[0], d->qpos[2], d->qpos[4], d->qpos[5], d->qpos[6];
    qa = qa_raw + qa_cal;  // Correct the angle. Make sure this only happens once per time step

    p << d->sensordata[0], d->sensordata[1], d->sensordata[2];
    Q.w() = d->sensordata[3];
    Q.x() = d->sensordata[4];
    Q.y() = d->sensordata[5];
    Q.z() = d->sensordata[6];
    v << d->sensordata[7], d->sensordata[8], d->sensordata[9];
    w << d->sensordata[10], d->sensordata[11], d->sensordata[12];

    if (ctrlMode == "Pos") {
      // if leg is using direct position controller, replace leg torque control values with qa_ref based pid torques
      // this is the simulated equivalent of using ODrive's position controller
      u(0) = -pid_q0Ptr->PIDControl(qa(0), qla_ref(0));
      u(1) = -pid_q2Ptr->PIDControl(qa(1), qla_ref(1));
    }

    sh = (d->sensordata[13] != 0);  // Contact detection, convert grf normal to bool
    // mj_step(m, d);
    mj_step1(m, d);
    d->ctrl[0] = u(0);  // moves joint 0
    d->ctrl[1] = u(1);  // moves joint 2
    d->ctrl[2] = u(2);  // moves rw0
    d->ctrl[3] = u(3);  // moves rw1
    d->ctrl[4] = u(4);  // moves rwz
    // d->qpos[0] = 0; // These seem to be buggy and ignore equality connect
    // d->qpos[2] = 0;
    mj_step2(m, d);

    mjtNum simstart = d->time;

    t_refresh += 1;

    if (t_refresh > refresh_rate) {
      std::cout << "sh = " << sh << "\n ";
      // std::cout << "contact = " << d->sensordata[13] << "\n ";
      // std::cout << "pos = " << qa_raw_(0) << ", " << qa_raw_(1) << ", " << qa_raw_(2) << ", " << qa_raw_(3) << ", " << qa_raw_(4) <<
      // "\n";
      // std::cout << "corrected pos = " << qa_(0) << ", " << qa_(1) << ", " << qa_(2) << ", " << qa_(3) << ", " << qa_(4) << "\n";
      // std::cout << "actuator force: " << (d->actuator_force[0]) << ", " << (d->actuator_force[1]) << ", " << (d->actuator_force[2]) << ",
      // "
      //           << (d->actuator_force[3]) << ", " << (d->actuator_force[4]) << ", " << (d->actuator_force[5]) << ", "
      //           << (d->actuator_force[6]) << std::endl;
      // std::cout << "ctrl: " << (d->ctrl[0]) << ", " << (d->ctrl[1]) << ", " << (d->ctrl[2]) << ", " << (d->ctrl[3]) << ", " <<
      // (d->ctrl[4])
      //           << ", " << (d->ctrl[5]) << ", " << (d->ctrl[6]) << std::endl;

      // visualization
      t_refresh = 0;  // reset refresh counter
      // 15 ms is a little smaller than 60 Hz.
      // std::this_thread::sleep_for(std::chrono::milliseconds(15));
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

  return retVals{p, Q, v, w, qa, dqa, sh};
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