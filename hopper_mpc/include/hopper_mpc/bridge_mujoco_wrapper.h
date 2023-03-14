#pragma once
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <stdbool.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

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

GLFWwindow* mujoco_init(char* directory_);

void mujoco_set_qpos(double p0, double p1, double p2);

void mujoco_set_qpos_joints(double p7, double p9);

struct fiveVals mujoco_get_qa();

struct fiveVals mujoco_get_dqa();

struct fiveVals mujoco_get_qa_fixed();
struct fiveVals mujoco_get_dqa_fixed();

struct quatVals mujoco_get_quat();

struct threeVals mujoco_get_p();
struct threeVals mujoco_get_v();

struct threeVals mujoco_get_wb();
struct fiveVals mujoco_get_tau();

struct fiveVals mujoco_get_rfx();

struct fiveVals mujoco_get_rfy();

struct fiveVals mujoco_get_rfz();

struct threeVals mujoco_get_ab();

struct threeVals mujoco_get_aef();

struct fiveVals mujoco_get_rtx();

struct fiveVals mujoco_get_rty();

struct fiveVals mujoco_get_rtz();

double mujoco_get_grf();
double mujoco_get_time();
void mujoco_update(GLFWwindow* window, double tau0, double tau1, double tau2, double tau3, double tau4);
void mujoco_end_sim();

#ifdef __cplusplus /* If this is a C++ compiler, end C linkage */
}
#endif