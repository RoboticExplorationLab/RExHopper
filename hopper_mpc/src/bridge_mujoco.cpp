#include "hopper_mpc/bridge_mujoco.h"
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "hopper_mpc/bridge_mujoco_wrapper.h"
// #include "hopper_mpc/bridge_mujoco_wrapper.c"

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

  // initialize mujoco
  mujoco_init(directory_);

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
    mujoco_set_qpos(model.p0_sit(0), model.p0_sit(1), model.p0_sit(2));
    mujoco_set_qpos_joints(model.qla_sit(0) - qa_cal(0), model.qla_sit(1) - qa_cal(1));
  } else {
    mujoco_set_qpos(model.p0(0), model.p0(1), model.p0(2));
  }
}

retVals MujocoBridge::SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) {
  // if leg is using direct position controller, replace leg torque control values with qa_ref based pid torques
  if (ctrlMode == "Pos") {  // this is the simulated equivalent of using ODrive's position controller
    u(0) = pid_q0Ptr->PIDControl(qa(0), qla_ref(0));
    u(1) = pid_q2Ptr->PIDControl(qa(1), qla_ref(1));
  }

  // get dqa
  fiveVals dqa_struct;
  fiveVals qa_struct;

  if (start == "fixed") {
    dqa_struct = mujoco_get_dqa_fixed();
  } else {
    dqa_struct = mujoco_get_dqa();
  }

  u *= -1;

  auto [tau0, i0, v0] = a0->Actuate(u(0), dqa_struct.q0);
  auto [tau1, i1, v1] = a1->Actuate(u(1), dqa_struct.q1);
  auto [tau2, i2, v2] = a2->Actuate(u(2), dqa_struct.q2);
  auto [tau3, i3, v3] = a3->Actuate(u(3), dqa_struct.q3);
  auto [tau4, i4, v4] = a4->Actuate(u(4), dqa_struct.q4);

  mujoco_update(tau0, tau1, tau2, tau3, tau4);

  // std::cout << "u before =  " << u.transpose() << "\n";

  // get measurements
  if (start == "fixed") {
    qa_struct = mujoco_get_qa_fixed();
    dqa_struct = mujoco_get_dqa_fixed();
  } else {
    qa_struct = mujoco_get_qa();
    dqa_struct = mujoco_get_dqa();
  }
  qa_raw << qa_struct.q0, qa_struct.q1, qa_struct.q2, qa_struct.q3, qa_struct.q4;
  dqa << dqa_struct.q0, dqa_struct.q1, dqa_struct.q2, dqa_struct.q3, dqa_struct.q4;

  qa = qa_raw + qa_cal;  // Correct the angle. Make sure this only happens once per time step
  // TODO: Should sensor stuff go after the integrate?

  auto [px, py, pz] = mujoco_get_p();
  p << px, py, pz;

  if (mujoco_get_time() == 0.001) {
    Q.setIdentity();  // sensordata quaternion initializes as all zeros which is invalid
  } else {
    auto [qw, qx, qy, qz] = mujoco_get_quat();
    Q.w() = qw;
    Q.x() = qx;
    Q.y() = qy;
    Q.z() = qz;
  }

  auto [vx, vy, vz] = mujoco_get_v();
  v << vx, vy, vz;

  auto [wx, wy, wz] = mujoco_get_wb();
  wb << wx, wy, wz;

  grf_normal = mujoco_get_grf();
  sh = grf_normal >= 60 ? true : false;  // check if contact is legit

  auto [taum0, taum1, taum2, taum3, taum4] = mujoco_get_tau();
  tau << taum0, taum1, taum2, taum3, taum4;
  tau_ref = u;

  // reaction forces
  auto [rfx0, rfx1, rfx2, rfx3, rfx4] = mujoco_get_rfx();
  rf_x << rfx0, rfx1, rfx2, rfx3, rfx4;
  auto [rfy0, rfy1, rfy2, rfy3, rfy4] = mujoco_get_rfy();
  rf_y << rfy0, rfy1, rfy2, rfy3, rfy4;
  auto [rfz0, rfz1, rfz2, rfz3, rfz4] = mujoco_get_rfz();
  rf_z << rfz0, rfz1, rfz2, rfz3, rfz4;

  // base accelerometer
  auto [ax, ay, az] = mujoco_get_ab();
  ab << ax, ay, az;

  // foot accelerometer
  auto [aex, aey, aez] = mujoco_get_aef();
  aef << aex, aey, aez;

  // reaction torques
  auto [rtx0, rtx1, rtx2, rtx3, rtx4] = mujoco_get_rtx();
  rt_x << rtx0, rtx1, rtx2, rtx3, rtx4;
  auto [rty0, rty1, rty2, rty3, rty4] = mujoco_get_rty();
  rt_y << rty0, rty1, rty2, rty3, rty4;
  auto [rtz0, rtz1, rtz2, rtz3, rtz4] = mujoco_get_rtz();
  rt_z << rtz0, rtz1, rtz2, rtz3, rtz4;

  return retVals{p, Q, v, wb, ab, aef, qa, dqa};
}

void MujocoBridge::End() {
  mujoco_end_sim();
}