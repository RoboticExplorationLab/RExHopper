#include "hopper_mpc/bridge_mujoco.h"
#include "mujoco/mujoco.h"
// for sleep timers
#include <chrono>
#include <thread>
#include "mujoco/glfw3.h"

char error[ERROR_SIZE] = "Could not load binary model";  // forced to make this global because I don't know how to code

MujocoBridge::MujocoBridge(Model model, float dt, float g, float mu, bool fixed, bool record) {
  // constructor
  model = model;
  dt = dt;
  g = g;
  mu = mu;
  fixed = fixed;
  record = record;
}

void MujocoBridge::Init() {
  // load and compile model
  // error_ = "Could not load binary model";

  m_ = mj_loadXML("/workspaces/RosDockerWorkspace/src/RExHopper/hopper_mpc/res/hopper_rev08/hopper_rev08.xml", 0, error, ERROR_SIZE);

  // make data
  d_ = mj_makeData(m_);

  // init GLFW
  if (!glfwInit()) mju_error("Could not initialize GLFW");

  // create window, make OpenGL context current, request v-sync
  window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);
  mjv_makeScene(m_, &scn_, 2000);               // space for 2000 objects
  mjr_makeContext(m_, &con_, mjFONTSCALE_150);  // model-specific context

  // initial position
  d_->qpos[0] = 1.57;

  // run main loop, target real-time simulation and 60 fps rendering
  timezero_ = d_->time;
  update_rate_ = dt;  // update rate is same as timestep size for now

  // making sure the first time step updates the ctrl previous_time
  // last_update = timezero - 1.0 / ctrl_update_freq;
}

void MujocoBridge::SimRun() {
  // use the first while condition if you want to simulate for a period.
  //    while( !glfwWindowShouldClose(window) and d->time-timezero < 1.5)
  if (!glfwWindowShouldClose(window_)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d_->time;
    while (d_->time - simstart < 1.0 / 60.0) mj_step(m_, d_);
    // 15 ms is a little smaller than 60 Hz.
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    // get framebuffer viewport
    viewport_ = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);

    // update scene and render
    mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport_, &scn_, &con_);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  } else {
  }
}

void MujocoBridge::End() {
  // free visualization storage
  mjv_freeScene(&scn_);
  mjr_freeContext(&con_);

  // free MuJoCo model and data, deactivate
  mj_deleteData(d_);
  mj_deleteModel(m_);
  mj_deactivate();

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
  ;
}