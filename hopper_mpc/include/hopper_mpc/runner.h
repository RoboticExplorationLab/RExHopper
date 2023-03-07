#pragma once
#include <memory>
#include "Eigen/Dense"
#include "hopper_mpc/bridge.h"
#include "hopper_mpc/bridge_hardware.h"
#include "hopper_mpc/bridge_mujoco.h"
// #include "hopper_mpc/bridge_raisim.h"
#include "hopper_mpc/filter.h"
#include "hopper_mpc/gait.h"
#include "hopper_mpc/kf.h"
#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/observer.h"
#include "hopper_mpc/rwa.h"

struct trajVals {
  std::vector<Eigen::Vector3d> p_refv;
  std::vector<Eigen::Vector3d> v_refv;
};

class Runner {  // The class

 public:  // Access specifier
  Runner(Model model_, double dt_, std::string bridge_, std::string start_, std::string ctrl_, int N_run_, bool plot_, bool skip_homing_,
         bool skip_kf_);  // constructor

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
  int N_stop;         // number of timesteps to "stop" at end of traj
  int N_sit;          // number of timesteps spent sitting at the beginning

  // --- if it is body frame it MUST have a b at the end of the name! --- //
  // --- otherwise assume world frame! --- //

  Eigen::Vector3d p;     // base world frame position
  Eigen::Quaterniond Q;  // base world frame quaternion
  Eigen::Vector3d v;     // base world frame velocity
  Eigen::Vector3d wb;    // base body frame rotational velocity
  Eigen::Vector3d w;     // base world frame rotational velocity
  Eigen::Vector3d ab;    // base body frame acceleration
  Eigen::Vector3d a;     // base world frame acceleration
  Eigen::Vector3d aeb;   // foot body frame acceleration
  Eigen::Vector3d ae;    // foot world frame acceleration

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

  Model model;
  double dt;           // timestep size
  std::string ctrl;    // controller
  std::string bridge;  // bridge (interface)
  std::string start;
  int N_run;  // number of timesteps in sim
  bool plot;
  bool skip_homing;
  bool skip_kf;
  double g;  // gravitational constant

  double x_adj;

  Eigen::VectorXd L;
  double h0;
  Eigen::Matrix3d J;
  double mu;

  std::unique_ptr<Bridge> bridgePtr;
  std::unique_ptr<Gait> gaitPtr;
  std::unique_ptr<Kf> kfPtr;
  std::unique_ptr<Observer> obPtr;
  std::unique_ptr<LowPass3D> lowpassPtr;

  std::shared_ptr<Leg> legPtr;
  std::shared_ptr<Rwa> rwaPtr;

  bool ContactSchedule(double t);
  std::vector<bool> ContactMap(int N, double dt, double t);
  std::vector<bool> ContactUpdate(std::vector<bool> C, int k);
  trajVals GenRefTraj(Eigen::Vector3d p_0, Eigen::Vector3d v_0, Eigen::Vector3d p_final);
  void UpdateGaitCycle(bool s, bool sh, double dz);
  int GaitCycleRef(double t);
  bool ContactCheck(bool sh, bool sh_prev, int k);
  bool FallCheck(Eigen::Quaterniond Q, double t);
};