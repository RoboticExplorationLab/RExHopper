#include "hopper_mpc/runner.h"
#include <Eigen/StdVector>
#include <chrono>  // for high_resolution_clock
#include <iostream>
#include "Eigen/Dense"
#include "hopper_mpc/plots.hpp"

Runner::Runner(Model model_, double dt_, std::string bridge_, std::string start_, std::string ctrl_, int N_run_, bool plot_,
               bool skip_homing_, bool skip_kf_) {
  model = model_;

  dt = dt_;

  bridge = bridge_;
  start = start_;
  ctrl = ctrl_;
  N_run = N_run_;

  plot = plot_;
  skip_homing = skip_homing_;
  skip_kf = skip_kf_;

  g = model.g;  // should g be defined here?
  L = model.leg_dim;
  h0 = 0.5;  // should match the number used in the mjcf!!
  J = model.inertia;
  mu = model.mu;

  n_X = 13;
  n_U = 6;

  t_p = 0.54;                        // gait period, seconds
  phi_switch = 0.5;                  // switching phase, must be between 0 and 1. Percentage of gait spent in contact.
  N = 40;                            // mpc prediction horizon length (mpc steps)
  dt_mpc = 0.01;                     // mpc sampling time (s), needs to be a factor of N
  N_mpc = dt_mpc / dt;               // mpc sampling time (timesteps), repeat mpc every x timesteps
  t_horizon = N * dt_mpc;            // mpc horizon time
  N_k = N * N_mpc;                   // total mpc prediction horizon length (low-level timesteps)
  t_start = 0.5 * t_p * phi_switch;  // start halfway through stance phase
  t_stance = t_p * phi_switch;       // time spent in stance
  N_c = t_stance / dt;               // number of timesteps spent in contact
  N_stop = 100;                      // number of timesteps spent stopped at the goal

  legPtr = std::make_shared<Leg>(model, dt);
  rwaPtr = std::make_shared<Rwa>(bridge, dt);

  // class definitions
  if (bridge_ == "hardware") {
    bridgePtr.reset(new HardwareBridge(model, dt, &legPtr, start, skip_homing));
    N_sit = 0;  // number of timesteps spent sitting
    x_adj = 0.005;
  } else if (bridge_ == "mujoco") {
    bridgePtr.reset(new MujocoBridge(model, dt, &legPtr, start, skip_homing));
    N_sit = 1500;  // number of timesteps spent sitting
    x_adj = -0.004;
    // } else if (bridge_ == "raisim") {
    // bridgePtr.reset(new RaisimBridge(model, dt, legPtr, start, skip_homing));
  } else {
    throw "Invalid bridge name! Use 'hardware' or 'mujoco'";
  }
  // TODO: This needs to be tied to sitting, not homing
  // initialize state
  if (skip_homing == false) {
    p = model.p0;
  } else {
    p = model.p0_sit;
  }

  Q.setIdentity();  // a vector expression of the coefficients (x,y,z,w)
  v.setZero();
  w.setZero();
  // initialize reference state
  p_ref << 2, 0, 0.5;
  Q_ref.setIdentity();
  v_ref.setZero();
  w_ref.setZero();

  // initialize gait cycle state
  gc_state = "Fall";
  gc_id = 3;
  /* Cmpr 0 0%-25%   | sh = 1
   Push 1 25%-50%  | sh = 1
   Rise 2 50%-75%  | sh = 0
   Fall 3 75%-100% | sh = 0 */
  // TODO: Make gc_state machine start when first contact is made
  ts = 0.875 * t_p;  // pretend you're starting halfway through 'fall'
  gc_state_prev = gc_state;

  ctrlMode = "Torque";                                               // "Torque&Pos"
  qla_ref = (model.S.transpose() * model.q_init).block<2, 1>(0, 0);  // convert from full joint space to actuated joint space
  peb_ref << 0, 0, -model.h0;                                        // desired operational space leg position in body frame
  veb_ref.setZero();                                                 // desired operational space leg velocity
  fb_ref.setZero();                                                  // desired operational space leg force
  u.setZero();                                                       // ctrl Torques

  gaitPtr.reset(new Gait(model, dt, peb_ref, &legPtr, &rwaPtr, x_adj));  // gait controller class
  kfPtr.reset(new Kf(dt));
  obPtr.reset(new Observer(dt, &legPtr));

  k_changed = 0;
  sh_saved = 0;
}

void Runner::Run() {
  bridgePtr->Init(x_adj);
  kfPtr->InitState(p, v, p + Q.matrix() * peb_ref, v + Q.matrix() * veb_ref);
  double t = 0.0;

  // initialize vectors of state history for plotting
  std::vector<std::vector<double>> p_raw_vec(N_run), v_raw_vec(N_run), pe_raw_vec(N_run), ve_raw_vec(N_run);
  std::vector<std::vector<double>> p_vec(N_run), v_vec(N_run), pe_vec(N_run), ve_vec(N_run);
  std::vector<std::vector<double>> theta_vec(N_run), theta_ref_vec(N_run), qa_vec(N_run), qa_ref_vec(N_run);
  std::vector<std::vector<double>> dqa_vec(N_run), dqa_ref_vec(N_run);
  std::vector<std::vector<double>> peb_vec(N_run), peb_ref_vec(N_run), tau_vec(N_run), tau_ref_vec(N_run), reactf_vec(N_run);
  std::vector<std::vector<double>> euler_vec(N_run), a_vec(N_run), ab_vec(N_run), ae_vec(N_run), grf_vec(N_run);
  std::vector<double> sh_hist(N_run), s_hist(N_run), gc_state_hist(N_run), gc_state_ref(N_run), grf_normal(N_run);

  int joint_id = 1;  // joint to check reaction forces at
  std::string rf_name = "Reaction Force on Joint " + std::to_string(joint_id) + " vs Timesteps";

  std::vector<bool> C(N_run);
  C = ContactMap(N_run, dt, t + ts);
  bool s = 0;
  bool sh = 0;

  trajVals trajvals = GenRefTraj(p, v, p_ref);
  auto p_refv = trajvals.p_refv;
  auto v_refv = trajvals.v_refv;

  double max_elapsed;
  auto t0_chrono = std::chrono::high_resolution_clock::now();  // Record start time

  Eigen::Vector3d grf(0, 0, 0);

  retVals retvals;
  uVals uvals;
  kfVals kfvals;

  Eigen::Quaterniond Q_offset;

  int k_final = N_run - 1;  // the final number of steps used, defaults to N_run -1

  for (int k = 0; k < N_run; k++) {
    // auto t_before = std::chrono::high_resolution_clock::now();  // time at beginning of loop

    retvals = bridgePtr->SimRun(u, qla_ref, ctrlMode);
    if (k == 0) {
      // TODO: rotate mocap position as well based on mocap yaw
      // Q_offset = Utils::ExtractYawQuat(retvals.Q).conjugate();
      Q_offset = retvals.Q.conjugate();  // required for fixed tests
    }
    Q = (Q_offset * retvals.Q).normalized();  // adjust yaw
    // Q = retvals.Q;

    bool stop = FallCheck(Q, t) + bridgePtr->stop;
    // bool stop = bridgePtr->stop;
    if (stop == true) {
      std::cout << "Stopping control loop \n";
      k_final = k;
      break;
    }

    wb = retvals.wb;
    w = Q.matrix() * wb;  // angular vel in the world frame
    ab = retvals.ab;
    a = Q.matrix() * ab;  // acceleration in the world frame

    qa = retvals.qa;
    dqa = retvals.dqa;  // note that this isn't being used in legPtr right now...

    sh = ContactCheck(bridgePtr->sh, sh_prev, k);  // TODO: stop using exclusively bridgeptr for this

    legPtr->UpdateState(qa.segment<2>(0), Q);                  // grab first two actuator pos values
    rwaPtr->UpdateState(qa.segment<3>(2), dqa.segment<3>(2));  // grab last three actuator pos and vel values
    Eigen::Vector3d peb = legPtr->GetPos();                    // pos of end-effector in body frame (P.E.B.)
    Eigen::Vector3d veb = legPtr->GetVel();                    // vel of end-effector in body frame (V.E.B.)

    Eigen::Matrix3d R2 = Utils::EulerToQuat(0.0, legPtr->q(2), 0.0).matrix();  // check to make sure this stuff works
    Eigen::Matrix3d R3 = Utils::EulerToQuat(0.0, legPtr->q(3), 0.0).matrix();
    aeb = R3 * (R2 * retvals.aef);
    ae = Q.matrix() * aeb;

    if (skip_kf == false) {
      // http://biorobotics.ri.cmu.edu/papers/paperUploads/Online_Kinematic_Calibration_for_Legged_Robots.pdf
      // get accurate velocity estimate while in contact using eq.20
      Eigen::Vector3d p_hat = retvals.p;
      Eigen::Vector3d v_hat_flight = retvals.v;                                     // world frame vel est in flight
      Eigen::Vector3d v_hat_contact = -Q.matrix() * (veb + Utils::Skew(wb) * peb);  // world frame vel est in contact
      Eigen::Vector3d v_hat = (1 - sh) * v_hat_flight + sh * v_hat_contact;   // switch b/t flight and contact version of vel estimation
      Eigen::Vector3d ve_hat = (1 - sh) * (v_hat_flight + Q.matrix() * veb);  // velocity of the foot in world frame (0 when in contact)
      Eigen::Vector3d pe_hat = p + Q.matrix() * peb;                          // position of the foot in world frame

      kfvals = kfPtr->EstUpdate(p_hat, v_hat, pe_hat, ve_hat, a, ae);  // use kalman filter
      p = kfvals.p;
      v = kfvals.v;
      pe = kfvals.pf;
      ve = kfvals.vf;

    } else {
      p = retvals.p;
      v = retvals.v;
      pe = p + Q.matrix() * peb;
      ve = v + Q.matrix() * veb;
    }

    s = C.at(k);  // bool s = ContactSchedule(t, 0);
    UpdateGaitCycle(s, sh, v(2));

    if (ctrl == "idle") {
      uvals = gaitPtr->Idle();  // warning: theta, etc. will not be plotted correctly with this
    } else if (ctrl == "circle") {
      uvals = gaitPtr->CircleTest();
    } else if (ctrl == "rotorvel") {
      uvals = gaitPtr->VelTest();
    } else if (ctrl == "rotorpos") {
      uvals = gaitPtr->PosTest();
    } else if (ctrl == "sit" || (start == "start_sit" && k <= N_sit)) {
      uvals = gaitPtr->Sit();
    } else if (start == "start_sit" && N_sit < k <= (N_sit + model.N_getup)) {
      uvals = gaitPtr->GetUp(Q);
    } else {
      if (ctrl == "raibert") {
        uvals = gaitPtr->Raibert(gc_state, gc_state_prev, p, Q, v, w, p_refv.at(k), Q_ref, v_refv.at(k), w_ref);
      } else if (ctrl == "stand") {
        uvals = gaitPtr->KinInvStand(Q);
      } else {
        throw "Invalid ctrl name! Use 'raibert', 'stand', 'sit', 'idle', or 'circle'";
        break;
      }
    }

    u = uvals.u;
    qla_ref = uvals.qla_ref;
    ctrlMode = uvals.ctrlMode;

    // clip torque commands to max
    for (int i = 0; i < model.n_a; i++) {
      u(i) = Utils::Clip(u(i), -model.a_tau_lim(i), model.a_tau_lim(i));
      // u(i) = Utils::Clip(u(i), -model.a_tau_stall(i) * 1.5, model.a_tau_stall(i) * 1.5);
    }

    // Eigen::Vector4d tau_dist = obPtr->TorqueEst(bridgePtr->tau.segment<2>(0));
    grf = obPtr->ForceEst(bridgePtr->tau.segment<2>(0));

    // previous
    gc_state_prev = gc_state;
    sh_prev = sh;

    if (plot == true) {
      p_raw_vec.at(k) = {retvals.p(0), retvals.p(1), retvals.p(2)};
      v_raw_vec.at(k) = {retvals.v(0), retvals.v(1), retvals.v(2)};
      Eigen::Vector3d pe_raw = retvals.p + Q.matrix() * peb;
      Eigen::Vector3d ve_raw = retvals.v + Q.matrix() * veb;
      pe_raw_vec.at(k) = {pe_raw(0), pe_raw(1), pe_raw(2)};
      ve_raw_vec.at(k) = {ve_raw(0), ve_raw(1), ve_raw(2)};

      p_vec.at(k) = {p(0), p(1), p(2)};
      v_vec.at(k) = {v(0), v(1), v(2)};
      pe_vec.at(k) = {pe(0), pe(1), pe(2)};
      ve_vec.at(k) = {ve(0), ve(1), ve(2)};

      Eigen::Vector3d euler = Utils::QuatToEuler(Q);
      euler_vec.at(k) = {euler(0) * 180 / M_PI, euler(1) * 180 / M_PI, euler(2) * 180 / M_PI};

      theta_vec.at(k) = {rwaPtr->theta(0) * 180 / M_PI, rwaPtr->theta(1) * 180 / M_PI, rwaPtr->theta(2) * 180 / M_PI};
      theta_ref_vec.at(k) = {rwaPtr->setp(0) * 180 / M_PI, rwaPtr->setp(1) * 180 / M_PI, rwaPtr->setp(2) * 180 / M_PI};

      qa_vec.at(k) = {qa(0) * 180 / M_PI, qa(1) * 180 / M_PI, qa(2) * 180 / M_PI, qa(3) * 180 / M_PI, qa(4) * 180 / M_PI};
      qa_ref_vec.at(k) = {qla_ref(0) * 180 / M_PI, qla_ref(1) * 180 / M_PI, rwaPtr->q_ref(0) * 180 / M_PI, rwaPtr->q_ref(1) * 180 / M_PI,
                          rwaPtr->q_ref(2) * 180 / M_PI};

      peb_vec.at(k) = {peb(0), peb(1), peb(2)};
      peb_ref_vec.at(k) = {gaitPtr->peb_ref(0), gaitPtr->peb_ref(1), gaitPtr->peb_ref(2)};

      tau_vec.at(k) = {bridgePtr->tau(0), bridgePtr->tau(1), bridgePtr->tau(2), bridgePtr->tau(3), bridgePtr->tau(4)};
      // NOTE: magnitude of tau_ref depends on whether gr is being handled by actuator.cpp or simulator/hardware
      tau_ref_vec.at(k) = {bridgePtr->tau_ref(0), bridgePtr->tau_ref(1), bridgePtr->tau_ref(2), bridgePtr->tau_ref(3),
                           bridgePtr->tau_ref(4)};
      // std::cout << dqa.transpose() << "\n";
      dqa_vec.at(k) = {dqa(0), dqa(1), dqa(2), dqa(3), dqa(4)};
      dqa_ref_vec.at(k) = {0.0, 0.0, rwaPtr->dq_ref(0), rwaPtr->dq_ref(1), rwaPtr->dq_ref(2)};

      reactf_vec.at(k) = {bridgePtr->rf_x(joint_id), bridgePtr->rf_y(joint_id), bridgePtr->rf_z(joint_id)};

      grf_vec.at(k) = {grf(0), grf(1), grf(2)};

      a_vec.at(k) = {a(0), a(1), a(2)};
      ab_vec.at(k) = {ab(0), ab(1), ab(2)};
      ae_vec.at(k) = {ae(0), ae(1), ae(2)};

      sh_hist.at(k) = sh;
      s_hist.at(k) = s;
      gc_state_hist.at(k) = gc_id;
      gc_state_ref.at(k) = GaitCycleRef(t + ts);  // the gait starts from ts, so t_actual = t + ts
      grf_normal.at(k) = bridgePtr->grf_normal;
    }

    t += dt;  // theoretical time

    // --- discount RTOS --- //
    // this would screw with simulator animation so only use for hardware
    if (bridge == "hardware") {
      auto t_after = std::chrono::high_resolution_clock::now();       // current time
      std::chrono::duration<double> tk_chrono = t_after - t0_chrono;  // measured time w.r.t. the initialization of loop
      if (tk_chrono.count() >= t) {
        // std::cout << "Missed 'real time' deadline at tk_chrono = " << tk_chrono.count() << ", t = " << t << " \n";
        // throw(std::runtime_error("Missed 'real time' deadline"));
      } else {
        int remainder = (t - tk_chrono.count()) * 1000;
        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(0, remainder - 1)));
      }
      // std::chrono::duration<double> elapsed = t_after - t_before;
      // std::cout << "Elapsed time: " << elapsed.count() << " s\n";
      // if (elapsed.count() > max_elapsed){
      //   max_elapsed = elapsed.count();
      // }
    }
    // --- end fake real-time --- //
  }
  std::cout << "End Control. \n";
  bridgePtr->End();

  // std::cout << "Max elapsed: " << max_elapsed << " s\n";

  if (plot == true) {
    // if (skip_kf == false) {
    //   Plots::Plot3(k_final, "Position, Filtered vs Raw", "p", p_vec, p_raw_vec, 0);
    //   Plots::Plot3(k_final, "Velocity, Filtered vs Raw", "v", v_vec, v_raw_vec, 0);
    //   Plots::Plot3(k_final, "Foot Position, Filtered vs Raw", "pe", pe_vec, pe_raw_vec, 0);
    //   Plots::Plot3(k_final, "Foot Velocity, Filtered vs Raw", "ve", ve_vec, ve_raw_vec, 0);
    // }
    // Plots::PlotMap2D(k_final, "2D Position vs Time", "p", p_vec, p_refv, 0, 0);
    // Plots::PlotMap3D(k_final, "3D Position vs Time", "p", p_vec, 0, 0);
    Plots::Plot5(k_final, "Actuator Joint Angular Positions", "q", qa_vec, qa_ref_vec, 0);
    Plots::Plot3(k_final, "Euler vs Time", "euler", euler_vec, euler_vec, 0);
    Plots::Plot3(k_final, "Theta vs Time", "theta", theta_vec, theta_ref_vec, 0);
    // Plots::Plot3(k_final, "Reaction Force vs Time", "joint " + std::to_string(joint_id), theta_vec, theta_ref_vec, 0);
    Plots::Plot5(k_final, "Tau vs Time", "tau", tau_vec, tau_ref_vec, 0);
    Plots::Plot5(k_final, "Dq vs Time", "dq", dqa_vec, dqa_ref_vec, 0);
    // Plots::Plot3(k_final, "Ground Reaction Force vs Time", "GRF", grf_vec, grf_vec, 0);

    // Plots::PlotMulti3(k_final, "Contact Timing", "Scheduled Contact", s_hist, "Sensed Contact", sh_hist, "Gait Cycle State",
    // gc_state_hist); Plots::PlotSingle(k_final, "Ground Reaction Force Normal", grf_normal);
    Plots::Plot3(k_final, "Measured Base Acceleration in World Frame", "acc", a_vec, a_vec, 0);
    Plots::Plot3(k_final, "Measured Base Acceleration in Body Frame", "acc", ab_vec, ab_vec, 0);
    // Plots::Plot3(k_final, "Measured Foot Acceleration", "acc", ae_vec, ae_vec, 0);
  }
}

void Runner::UpdateGaitCycle(bool s, bool sh, double dz) {
  if (gc_state == "Cmpr" && dz >= 0) {
    gc_state = "Push";
    gc_id = 1;
  } else if (gc_state == "Push" && s == false && sh == false) {  // s == false &&
    gc_state = "Rise";
    gc_id = 2;
  } else if (gc_state == "Rise" && dz <= 0) {
    gc_state = "Fall";
    gc_id = 3;
  } else if (gc_state == "Fall" && sh == true) {
    gc_state = "Cmpr";
    gc_id = 0;
  }
}

/* Cmpr 0 0%-25%   | sh = 1
   Push 1 25%-50%  | sh = 1
   Rise 2 50%-75%  | sh = 0
   Fall 3 75%-100% | sh = 0 */

int Runner::GaitCycleRef(double t) {
  // this makes a lot of assumptions about the gait and phi_switch. Modify if phi_switch is modified
  double phi = std::fmod(t / t_p, 1);
  int gc_id_ref;
  if (abs(phi) < 0.25) {
    gc_id_ref = 0;
  } else if (abs(phi) < 0.5) {
    gc_id_ref = 1;
  } else if (abs(phi) < 0.75) {
    gc_id_ref = 2;
  } else {
    gc_id_ref = 3;
  }
  return gc_id_ref;
}

bool Runner::ContactSchedule(double t) {
  // used to generate contact map
  double phi = std::fmod(t / t_p, 1);
  // std::cout << phi << "\n";
  return phi < phi_switch ? true : false;
}

std::vector<bool> Runner::ContactMap(int N, double dt, double t) {
  // generate vector of scheduled contact states over the prediction horizon
  std::vector<bool> C(N);
  for (int k = 0; k < N; k++) {
    C.at(k) = ContactSchedule(t);
    // std::cout << C.at(k) << "\n";
    t += dt;
  }
  return C;
}

bool Runner::ContactCheck(bool sh, bool sh_prev, int k) {
  // if contact has just been made, freeze contact detection to True for x timesteps
  // or if contact has just been lost, freeze contact detection to False for x timesteps
  // protects against vibration/bouncing-related bugs
  int n_ts = 0.05 / dt;
  if ((sh_prev != sh) && (k - k_changed > n_ts)) {
    k_changed = k;
    sh_saved = sh;
  }
  if (k - k_changed <= n_ts) {
    sh = sh_saved;
  }
  return sh;
}

trajVals Runner::GenRefTraj(Eigen::Vector3d p_0, Eigen::Vector3d v_0, Eigen::Vector3d p_final) {
  std::size_t N_traj = N_run - N_stop;
  std::size_t N_ref = N_run + N_k;
  std::vector<Eigen::Vector3d> p_refv(N_traj);
  std::vector<Eigen::Vector3d> v_refv(N_traj);

  Eigen::VectorXd px_traj = Eigen::VectorXd::LinSpaced(N_traj, p_0(0), p_final(0));
  Eigen::VectorXd py_traj = Eigen::VectorXd::LinSpaced(N_traj, p_0(1), p_final(1));
  Eigen::VectorXd pz_traj = Eigen::VectorXd::LinSpaced(N_traj, p_0(2), p_final(2));
  Eigen::VectorXd vx_traj;
  Eigen::VectorXd vy_traj;
  Eigen::VectorXd vz_traj;
  vx_traj.resize(N_traj);
  vy_traj.resize(N_traj);
  vz_traj.resize(N_traj);
  for (int i = 0; i < (N_traj - 1); i++) {
    vx_traj(i) = (px_traj(i + 1) - px_traj(i)) / dt;
    vy_traj(i) = (py_traj(i + 1) - py_traj(i)) / dt;
    vz_traj(i) = (pz_traj(i + 1) - pz_traj(i)) / dt;
  }

  for (int i = 0; i < N_traj; i++) {
    // interpolate positions
    p_refv.at(i) << px_traj(i), py_traj(i), pz_traj(i);
    v_refv.at(i) << vx_traj(i), vy_traj(i), vz_traj(i);
    // w_refv.at(i) << px_traj(i), py_traj(i), pz_traj(i);
  }
  v_refv.back() << 0, 0, 0;  // Make sure value of v_refv for final timestep is zeros

  std::vector<Eigen::Vector3d> p_sitv(N_stop + N_k);
  std::vector<Eigen::Vector3d> v_sitv(N_stop + N_k);
  for (int i = 0; i < N_stop + N_k; i++) {
    p_sitv.push_back(p_refv.back());  // add final value
    v_sitv.push_back(v_refv.back());  // add final value
  }
  p_refv.insert(p_refv.end(), p_sitv.begin(), p_sitv.end());
  v_refv.insert(v_refv.end(), v_sitv.begin(), v_sitv.end());

  return trajVals{p_refv, v_refv};  // TODO: sine wave for mpc
}

bool Runner::FallCheck(Eigen::Quaterniond Q, double t) {
  Eigen::Quaterniond Q_frame;
  Q_frame.setIdentity();
  Eigen::Quaterniond Q_no_yaw;
  Q_no_yaw = (Utils::ExtractYawQuat(Q).conjugate() * Q).normalized();  // the base quaternion ignoring heading
  bool stop = false;
  double angle = Utils::AngleBetween(Q_frame, Q_no_yaw) * 180 / M_PI;
  // angle max = arcsin(tau_max/(m * g * l)) -> to degrees
  if (angle > 4.875) {
    std::cout << "FallCheck: Fall likely; Deactivating at t = " << t << " s with angle = " << angle << " degrees \n";
    stop = true;
  }
  return stop;
};
