#pragma once

struct fiveVals {  // Declare a local structure
  double q0;
  double q1;
  double q2;
  double q3;
  double q4;
};

struct quatVals {
  double qw;
  double qx;
  double qy;
  double qz;
};

struct threeVals {
  double x;
  double y;
  double z;
};

void mujoco_init(char* directory_);

void mujoco_set_qpos(double p0, double p1, double p2);

void mujoco_set_qpos_joints(double p7, double p9);

fiveVals mujoco_get_qa();

fiveVals mujoco_get_dqa();

fiveVals mujoco_get_qa_fixed();
fiveVals mujoco_get_dqa_fixed();

quatVals mujoco_get_quat();

threeVals mujoco_get_p();
threeVals mujoco_get_v();

threeVals mujoco_get_wb();
fiveVals mujoco_get_tau();

fiveVals mujoco_get_rfx();

fiveVals mujoco_get_rfy();

fiveVals mujoco_get_rfz();

threeVals mujoco_get_ab();

threeVals mujoco_get_aef();

fiveVals mujoco_get_rtx();

fiveVals mujoco_get_rty();

fiveVals mujoco_get_rtz();

double mujoco_get_grf();
double mujoco_get_time();
void mujoco_update(double tau0, double tau1, double tau2, double tau3, double tau4);
void mujoco_end_sim();