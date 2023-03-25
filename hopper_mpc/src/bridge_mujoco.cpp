#include "hopper_mpc/bridge_mujoco.h"
#include <mujoco/mujoco.h>
#include <unistd.h>  // for getcwd()
#include <chrono>    // for sleep timers
#include <iostream>
#include <thread>

mjModel* MujocoBridge::m;      // MuJoCo model
mjData* MujocoBridge::data_m;  // MuJoCo data
mjvCamera MujocoBridge::cam;   // abstract camera
mjvOption MujocoBridge::opt;   // visualization options
mjvScene MujocoBridge::scn;    // abstract scene
mjrContext MujocoBridge::con;  // custom GPU context
                               // mouse interaction
bool MujocoBridge::button_left;
bool MujocoBridge::button_middle;
bool MujocoBridge::button_right;
double MujocoBridge::lastx;
double MujocoBridge::lasty;

MujocoBridge::MujocoBridge(Model model_, double dt_, std::shared_ptr<Leg>* legPtr_, std::string start_, bool skip_homing_)
    : Base(model_, dt_, legPtr_, start_, skip_homing_) {}

void MujocoBridge::Init(double x_adj_) {
  char error[ERROR_SIZE] = "Could not load binary model";

  std::string path_mjcf;
  if (start == "fixed") {
    path_mjcf = model.mjcf_fixed_path;
  } else {
    path_mjcf = model.mjcf_path;
  }
  char* cwd;
  char buff[PATH_MAX + 1];
  cwd = getcwd(buff, PATH_MAX + 1);  // get current working directory
  std::string path_cwd = cwd;
  str = path_cwd + path_mjcf;  // combine current directory with relative path for mjcf

  char* directory_ = new char[PATH_MAX + 1];  // just needs to be larger than the actual string
  strcpy(directory_, str.c_str());            // converting back to char... probably wasteful but whatevs
  if (std::strlen(directory_) > 4 && !std::strcmp(directory_ + std::strlen(directory_) - 4, ".mjb")) {
    m = mj_loadModel(directory_, 0);
  } else {
    m = mj_loadXML(directory_, 0, error, 1000);
  }
  if (!m) {
    mju_error_s("Load model error: %s", error);
  }

  data_m = mj_makeData(m);  // MuJoCo data

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
  timezero = data_m->time;
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

  if (start == "start_sit") {  // the robot starts from a sitting position
    data_m->qpos[0] = model.p0_sit(0);
    data_m->qpos[1] = model.p0_sit(1);
    data_m->qpos[2] = model.p0_sit(2);
    data_m->qpos[7] = model.qla_sit(0) - qa_cal(0);  // joint 0
    data_m->qpos[9] = model.qla_sit(1) - qa_cal(1);  // joint 2
  } else {
    data_m->qpos[0] = model.p0(0);
    data_m->qpos[1] = model.p0(1);
    data_m->qpos[2] = model.p0(2);
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
    if (start == "fixed") {
      dqa << data_m->qvel[0], data_m->qvel[2], data_m->qvel[4], data_m->qvel[5], data_m->qvel[6];
    } else {
      dqa << data_m->qvel[6], data_m->qvel[8], data_m->qvel[10], data_m->qvel[11], data_m->qvel[12];
    }

    u *= -1;
    // std::cout << "u before =  " << u.transpose() << "\n";
    mj_step1(m, data_m);  // mj_step(m, d);
    auto [tau0, i0, v0] = a0->Actuate(u(0), dqa(0));
    auto [tau1, i1, v1] = a1->Actuate(u(1), dqa(1));
    auto [tau2, i2, v2] = a2->Actuate(u(2), dqa(2));
    auto [tau3, i3, v3] = a3->Actuate(u(3), dqa(3));
    auto [tau4, i4, v4] = a4->Actuate(u(4), dqa(4));
    data_m->ctrl[0] = tau0;  // moves joint 0
    data_m->ctrl[1] = tau1;  // moves joint 2
    data_m->ctrl[2] = tau2;  // moves rw0
    data_m->ctrl[3] = tau3;  // moves rw1
    data_m->ctrl[4] = tau4;  // moves rwz
    mj_step2(m, data_m);
    // std::cout << "tau = " << tau0 << ", " << tau1 << ", " << tau2 << ", " << tau3 << ", " << tau4 << "\n";

    // get measurements
    if (start == "fixed") {
      qa_raw << data_m->qpos[0], data_m->qpos[2], data_m->qpos[4], data_m->qpos[5], data_m->qpos[6];
      dqa << data_m->qvel[0], data_m->qvel[2], data_m->qvel[4], data_m->qvel[5], data_m->qvel[6];
    } else {
      qa_raw << data_m->qpos[7], data_m->qpos[9], data_m->qpos[11], data_m->qpos[12], data_m->qpos[13];
      dqa << data_m->qvel[6], data_m->qvel[8], data_m->qvel[10], data_m->qvel[11], data_m->qvel[12];
      // first seven indices are for base pos and quat in floating body mode
      // for velocities, it's the first six indices
    }

    qa = qa_raw + qa_cal;  // Correct the angle. Make sure this only happens once per time step
    // TODO: Should sensor stuff go after the integrate?
    p << data_m->sensordata[0], data_m->sensordata[1], data_m->sensordata[2];

    if (data_m->time == 0.001) {
      Q.setIdentity();  // sensordata quaternion initializes as all zeros which is invalid
    } else {
      Q.w() = data_m->sensordata[3];
      Q.x() = data_m->sensordata[4];
      Q.y() = data_m->sensordata[5];
      Q.z() = data_m->sensordata[6];
    }

    v << data_m->sensordata[7], data_m->sensordata[8], data_m->sensordata[9];

    wb << data_m->sensordata[10], data_m->sensordata[11], data_m->sensordata[12];

    grf_normal = data_m->sensordata[13];
    sh = grf_normal >= 60 ? true : false;  // check if contact is legit

    tau << data_m->sensordata[14], data_m->sensordata[15], data_m->sensordata[16], data_m->sensordata[17], data_m->sensordata[18];
    tau_ref = u;

    rf_x << data_m->sensordata[19], data_m->sensordata[22], data_m->sensordata[25], data_m->sensordata[28], data_m->sensordata[31];
    rf_y << data_m->sensordata[20], data_m->sensordata[23], data_m->sensordata[26], data_m->sensordata[29], data_m->sensordata[32];
    rf_z << data_m->sensordata[21], data_m->sensordata[24], data_m->sensordata[27], data_m->sensordata[30], data_m->sensordata[33];

    ab << data_m->sensordata[34], data_m->sensordata[35], data_m->sensordata[36];   // base accelerometer
    aef << data_m->sensordata[37], data_m->sensordata[38], data_m->sensordata[39];  // foot accelerometer

    rt_x << data_m->sensordata[40], data_m->sensordata[43], data_m->sensordata[46], data_m->sensordata[49], data_m->sensordata[52];
    rt_y << data_m->sensordata[41], data_m->sensordata[44], data_m->sensordata[47], data_m->sensordata[50], data_m->sensordata[53];
    rt_z << data_m->sensordata[42], data_m->sensordata[45], data_m->sensordata[48], data_m->sensordata[51], data_m->sensordata[54];

    t_refresh += 1;
    if (t_refresh > refresh_rate) {
      // visualization
      t_refresh = 0;  // reset refresh counter
      // 15 ms is a little smaller than 60 Hz.
      // get framebuffer viewport
      viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

      // update scene and render
      mjv_updateScene(m, data_m, &opt, NULL, &cam, mjCAT_ALL, &scn);
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
  mj_deleteData(data_m);
  mj_deleteModel(m);
  mj_deactivate();

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
  ;
}
