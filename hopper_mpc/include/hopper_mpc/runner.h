#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/bridge_hardware.h"
#include "hopper_mpc/bridge_mujoco.h"
// #include "hopper_mpc/bridge_raisim.h"
#include "hopper_mpc/gait.h"
#include "hopper_mpc/kf.h"
#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/rwa.h"

class Runner {  // The class

 public:  // Access specifier
  Runner(Model model_, int N_run_, double dt_, std::string ctrl_, std::string bridge_, bool plot_, bool fixed_, bool spr_, bool record_,
         bool ignore_kf_);  // constructor

  void Run();

 private:
  double t_p;         // gait period, seconds
  double phi_switch;  // switching phase, must be between 0 and 1. Percentage of gait spent in contact.
  int N;              // mpc prediction horizon length (mpc steps)
  double dt_mpc;      // mpc sampling time (s), needs to be a factor of N
  int N_mpc;          // mpc sampling time (timesteps), repeat mpc every x timesteps
  double t_horizon;   // mpc horizon time
  int N_k;            // total mpc prediction horizon length (low-level timesteps)
  double t_start;     // start halfway through stance phase
  double t_stance;    // time spent in stance
  int N_c;            // number of timesteps spent in contact
  int N_sit;          // number of timesteps to "sit" at end of traj

  // --- if it is body frame it MUST have a b at the end of the name! --- //
  // --- otherwise assume world frame! --- //

  Eigen::Vector3d p;     // base world frame position
  Eigen::Quaterniond Q;  // base world frame quaternion
  Eigen::Vector3d v;     // base world frame velocity
  Eigen::Vector3d wb;    // base body frame rotational velocity
  Eigen::Vector3d w;     // base world frame rotational velocity
  Eigen::Vector3d ab;    // base body frame acceleration
  Eigen::Vector3d a;     // base world frame acceleration

  Eigen::Vector3d p_ref;     // base world frame position
  Eigen::Quaterniond Q_ref;  // base world frame quaternion
  Eigen::Vector3d v_ref;     // base world frame velocity
  Eigen::Vector3d w_ref;     // base world frame rotational velocity

  Eigen::Matrix<double, 5, 1> qa;
  Eigen::Matrix<double, 5, 1> dqa;

  std::string gc_state;  // gait cycle state
  std::string gc_state_prev;
  int gc_id;

  Eigen::Matrix<double, 5, 1> u;        // control torques
  Eigen::Matrix<double, 2, 1> qla_ref;  // leg actuator position setpoints
  std::string ctrlMode;
  Eigen::Vector3d pe;
  Eigen::Vector3d ve;
  Eigen::Vector3d peb_ref;  // body frame end effector position reference
  Eigen::Vector3d veb_ref;  // body frame end effector vel reference
  Eigen::Vector3d fb_ref;   // body frame end effector force ref

  int n_X;  // number of sim states
  int n_U;  // number of sim controls

  double ts;  // starting time

  // contact checker variables
  int k_changed;
  bool sh_saved;
  bool sh_prev;

  // circletest vars
  double x1;
  double z1;
  double z;
  double r;
  int flip;

  Model model;
  int N_run;           // number of timesteps in sim
  double dt;           // timestep size
  std::string ctrl;    // controller
  std::string bridge;  // bridge (interface)
  bool plot;
  bool fixed;
  bool spr;
  bool record;
  bool ignore_kf;
  double g;  // gravitational constant

  Eigen::VectorXd L;
  double h0;
  Eigen::Matrix3d J;
  double mu;

  std::unique_ptr<Bridge> bridgePtr;
  std::unique_ptr<Gait> gaitPtr;
  std::unique_ptr<Kf> kfPtr;

  std::shared_ptr<Leg> legPtr;
  std::shared_ptr<Rwa> rwaPtr;

  bool ContactSchedule(double t);
  std::vector<bool> ContactMap(int N, double dt, double t);
  std::vector<bool> ContactUpdate(std::vector<bool> C, int k);
  trajVals GenRefTraj(Eigen::Vector3d p_0, Eigen::Vector3d v_0, Eigen::Vector3d p_final);
  void GaitCycleUpdate(bool s, bool sh, double dz);
  int GaitCycleRef(double t);
  bool ContactCheck(bool sh, bool sh_prev, int k);
  void CircleTest();
};