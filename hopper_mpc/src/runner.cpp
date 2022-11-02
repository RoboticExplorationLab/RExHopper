#include "hopper_mpc/runner.h"
#include <Eigen/StdVector>
#include <chrono>  // for high_resolution_clock
#include <iostream>
#include "Eigen/Dense"
#include "hopper_mpc/plots.hpp"

Runner::Runner(Model model_, int N_run_, double dt_, std::string ctrl_, std::string bridge_, bool plot_, bool fixed_, bool spr_,
               bool record_, bool skip_kf_) {
  model = model_;
  N_run = N_run_;
  dt = dt_;
  ctrl = ctrl_;
  bridge = bridge_;
  plot = plot_;
  fixed = fixed_;
  spr = spr_;
  record = record_;
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
  N_sit = 100;                       // number of timesteps spent sitting at the goal

  // class definitions
  if (bridge_ == "hardware") {
    bridgePtr.reset(new HardwareBridge(model, dt, fixed, record));
  } else if (bridge_ == "mujoco") {
    bridgePtr.reset(new MujocoBridge(model, dt, fixed, record));
    // } else if (bridge_ == "raisim") {
    // bridgePtr.reset(new RaisimBridge(model, dt, fixed, record));
  } else {
    throw "Invalid bridge name! Use 'hardware' or 'mujoco'";
  }
  // initialize state
  p << 0, 0, 0.5;  // must match starting position in mjcf!!
  Q.coeffs() << 0, 0, 0, 1;
  v.setZero();
  w.setZero();
  // initialize reference state
  p_ref << 2, 0, 0.5;
  Q_ref.coeffs() << 0, 0, 0, 1;
  v_ref.setZero();
  w_ref.setZero();

  // initialize gait cycle state
  gc_state = "Fall";
  gc_id = 3;
  /* Cmpr 0 0%-25%   | sh = 1
   Push 1 25%-50%  | sh = 1
   Rise 2 50%-75%  | sh = 0
   Fall 3 75%-100% | sh = 0 */
  ts = 0.875 * t_p;  // pretend you're starting halfway through 'fall'
  gc_state_prev = gc_state;

  ctrlMode = "Torque";                                               // "Torque&Pos"
  qla_ref = (model.S.transpose() * model.q_init).block<2, 1>(0, 0);  // convert from full joint space to actuated joint space
  qla_ref << qla_ref(0), qla_ref(1) + 0.5;
  peb_ref << 0, 0, -model.h0 * 2;  // desired operational space leg position in body frame
  veb_ref.setZero();               // desired operational space leg velocity
  fb_ref.setZero();                // desired operational space leg force
  u.setZero();                     // ctrl Torques
  legPtr = std::make_shared<Leg>(model, dt);
  rwaPtr = std::make_shared<Rwa>(dt);

  gaitPtr.reset(new Gait(model, dt, peb_ref, &legPtr, &rwaPtr));  // gait controller class
  kfPtr.reset(new Kf(dt));

  k_changed = 0;
  sh_saved = 0;

  // variables for CircleTest
  x1 = 0;
  z1 = -0.4;
  z = -0.3;  // peb_ref(2);
  r = 0.1;
  flip = -1;
}

void Runner::Run() {  // Method/function defined inside the class
  bridgePtr->Init();
  double t = 0.0;

  std::vector<double> theta_x(N_run), theta_y(N_run), theta_z(N_run), setp_x(N_run), setp_y(N_run), setp_z(N_run), q0(N_run), q2(N_run),
      q0_ref(N_run), q2_ref(N_run), peb_x(N_run), peb_z(N_run), peb_refx(N_run), peb_refz(N_run), tau_0(N_run), tau_1(N_run), tau_2(N_run),
      tau_3(N_run), tau_4(N_run), tau_ref0(N_run), tau_ref1(N_run), tau_ref2(N_run), tau_ref3(N_run), tau_ref4(N_run), dq_0(N_run),
      dq_1(N_run), dq_2(N_run), dq_3(N_run), dq_4(N_run), dq_ref0(N_run), dq_ref1(N_run), dq_ref2(N_run), dq_ref3(N_run), dq_ref4(N_run),
      p_z(N_run), p_refz(N_run), sh_hist(N_run), s_hist(N_run), gc_state_hist(N_run), gc_state_ref(N_run), grf_normal(N_run);

  std::vector<double> rfx(N_run), rfy(N_run), rfz(N_run);
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
  retVals retvals;
  uVals uvals;
  kfVals kfvals;

  for (int k = 0; k < N_run; k++) {
    auto t_before = std::chrono::high_resolution_clock::now();  // time at beginning of loop

    retvals = bridgePtr->SimRun(u, qla_ref, ctrlMode);  // still need c, tau, i, v, grf
    Q = retvals.Q;
    wb = retvals.wb;
    w = Q.matrix() * wb;  // angular vel in the world frame
    ab = retvals.ab;
    a = Q.matrix() * ab;  // acceleration in the world frame

    qa = retvals.qa;
    dqa = retvals.dqa;
    sh = ContactCheck(retvals.sh, sh_prev, k);
    legPtr->UpdateState(qa.block<2, 1>(0, 0), Q);  // grab first two actuator pos values
    Eigen::Vector3d peb = legPtr->KinFwd();        // pos of end-effector in body frame (P.E.B.)
    Eigen::Vector3d veb = legPtr->GetVel();        // vel of end-effector in body frame (V.E.B.)

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

      kfvals = kfPtr->EstUpdate(p_hat, v_hat, pe_hat, ve_hat, Q, a, ae, sh);  // use kalman filter
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

    s = C.at(k);                   // bool s = ContactSchedule(t, 0);
    GaitCycleUpdate(s, sh, v(2));  // TODO: should use v_global instead?

    if (ctrl == "raibert") {
      uvals = gaitPtr->uRaibert(gc_state, gc_state_prev, p, Q, v, w, p_refv.at(k), Q_ref, v_refv.at(k), w_ref);
      u = uvals.u;
      qla_ref = uvals.qla_ref;
      ctrlMode = uvals.ctrlMode;
    } else if (ctrl == "stand") {
      uvals = gaitPtr->uKinInvStand(gc_state, gc_state_prev, p, Q, v, w, p_ref, Q_ref, v_ref, w_ref);
      u = uvals.u;
      qla_ref = uvals.qla_ref;
      ctrlMode = uvals.ctrlMode;
    } else if (ctrl == "idle") {
      u << 0, 0, 0, 0, 0;  // do nothing
      ctrlMode = "Torque";
    } else if (ctrl == "circle") {
      CircleTest();  // edits peb_ref in-place
      qla_ref = legPtr->KinInv(peb_ref);
      ctrlMode = "Pos";
    } else {
      throw "Invalid ctrl name! Use 'raibert', 'stand', 'idle', or 'circle'";
    }

    gc_state_prev = gc_state;  // should be last // u << 0.1, 0.1, 0.1, 0.1, 0.1;
    sh_prev = sh;

    if (plot == true) {
      theta_x.at(k) = rwaPtr->theta(0);
      theta_y.at(k) = rwaPtr->theta(1);
      theta_z.at(k) = rwaPtr->theta(2);
      setp_x.at(k) = rwaPtr->setp(0);
      setp_y.at(k) = rwaPtr->setp(1);
      setp_z.at(k) = rwaPtr->setp(2);
      q0.at(k) = qa(0) * 180 / M_PI;
      q2.at(k) = qa(1) * 180 / M_PI;
      q0_ref.at(k) = qla_ref(0) * 180 / M_PI;
      q2_ref.at(k) = qla_ref(1) * 180 / M_PI;

      peb_x.at(k) = peb(0);
      peb_z.at(k) = peb(2);
      peb_refx.at(k) = gaitPtr->peb_ref(0);
      peb_refz.at(k) = gaitPtr->peb_ref(2);

      tau_0.at(k) = bridgePtr->tau(0);
      tau_1.at(k) = bridgePtr->tau(1);
      tau_2.at(k) = bridgePtr->tau(2);
      tau_3.at(k) = bridgePtr->tau(3);
      tau_4.at(k) = bridgePtr->tau(4);
      // NOTE: magnitude of tau_ref depends on whether gr is being handled by actuator.cpp or simulator/hardware
      tau_ref0.at(k) = bridgePtr->tau_ref(0);
      tau_ref1.at(k) = bridgePtr->tau_ref(1);
      tau_ref2.at(k) = bridgePtr->tau_ref(2);
      tau_ref3.at(k) = bridgePtr->tau_ref(3);
      tau_ref4.at(k) = bridgePtr->tau_ref(4);

      dq_0.at(k) = dqa(0);
      dq_1.at(k) = dqa(1);
      dq_2.at(k) = dqa(2);
      dq_3.at(k) = dqa(3);
      dq_4.at(k) = dqa(4);

      p_z.at(k) = p(2);
      p_refz.at(k) = p_ref(2);
      sh_hist.at(k) = sh;
      s_hist.at(k) = s;

      gc_state_hist.at(k) = gc_id;
      gc_state_ref.at(k) = GaitCycleRef(t + ts);  // the gait starts from ts, so t_actual = t + ts
      grf_normal.at(k) = bridgePtr->grf_normal;
      rfx.at(k) = bridgePtr->rf_x(joint_id);
      rfy.at(k) = bridgePtr->rf_y(joint_id);
      rfz.at(k) = bridgePtr->rf_z(joint_id);
    }

    t += dt;  // theoretical time
    // this would screw with simulator animation so only use for hardware
    if (bridge == "hardware") {
      auto t_after = std::chrono::high_resolution_clock::now();       // current time
      std::chrono::duration<double> tk_chrono = t_after - t0_chrono;  // measured time w.r.t. the initialization of loop
      if (tk_chrono.count() >= t) {
        std::cout << "Missed 'real time' deadline at tk_chrono = " << tk_chrono.count() << ", t = " << t << " \n";
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
  }
  std::cout << "End Control. \n";
  bridgePtr->End();

  // std::cout << "Max elapsed: " << max_elapsed << " s\n";

  if (plot == true) {
    // Plots::Grf(N_run, grf_normal);
    Plots::OpSpacePos(N_run, peb_x, peb_z, peb_refx, peb_refz);
    Plots::Plot2(N_run, "Joint Angular Positions", "q0", q0, q0_ref, "q2", q2, q2_ref, 0);
    Plots::Plot3(N_run, "Contact Timing", "Body Z Pos", p_z, p_refz, "Contact", sh_hist, s_hist, "Gait Cycle State", gc_state_hist,
                 gc_state_ref, 0);
    // Plots::Plot3(N_run, "Theta vs Timesteps", "Theta_x", theta_x, setp_x, "Theta_y", theta_y, setp_y, "Theta_z", theta_z, setp_z, 0);
    // Plots::Plot5(N_run, "Tau vs Timesteps", "Tau_0", tau_0, tau_ref0, "Tau_1", tau_1, tau_ref1, "Tau_2", tau_2, tau_ref2, "Tau_3", tau_3,
    //              tau_ref3, "Tau_4", tau_4, tau_ref4, 60);
    // Plots::Plot5(N_run, "dq vs Timesteps", "dq_0", dq_0, dq_0, "dq_1", dq_1, dq_1, "dq_2", dq_2, dq_2, "dq_3", dq_3, dq_3, "dq_4", dq_4,
    //              dq_4, 0);
    // Plots::Plot3(N_run, rf_name, "F_x", rfx, rfx, "F_y", rfy, rfy, "F_z", rfz, rfz, 0);
  }
}

void Runner::GaitCycleUpdate(bool s, bool sh, double dz) {
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

std::vector<bool> Runner::ContactUpdate(std::vector<bool> C, int k) {
  // shift contact map. Use if contact has been made early or was previously late
  double t_shifted = 0;
  for (int i = k; i < N_run; i++) {
    C.at(i) = ContactSchedule(t_shifted);
    t_shifted += dt;
  }
  // self.k_f_update(C)  // update kf_list
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
  std::size_t N_traj = N_run - N_sit;
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

  std::vector<Eigen::Vector3d> p_sitv(N_sit + N_k);
  std::vector<Eigen::Vector3d> v_sitv(N_sit + N_k);
  for (int i = 0; i < N_sit + N_k; i++) {
    p_sitv.push_back(p_refv.back());  // add final value
    v_sitv.push_back(v_refv.back());  // add final value
  }
  p_refv.insert(p_refv.end(), p_sitv.begin(), p_sitv.end());
  v_refv.insert(v_refv.end(), v_sitv.begin(), v_sitv.end());

  return trajVals{p_refv, v_refv};  // TODO: sine wave for mpc
}

void Runner::CircleTest() {
  // edits peb_ref in-place
  z += 0.0005 * flip;  // std::cout << z << "\n";
  if (z <= -0.5 || z >= -0.3) {
    flip *= -1;
  }
  double x = (sqrt(pow(r, 2) - pow(z - z1, 2)) + x1) * -flip;
  if (isnan(x)) {
    x = 0;
  }
  peb_ref(0) = x;
  peb_ref(2) = z;

  // std::cout << "peb_ref = " << peb_ref(0) << ", " << peb_ref(1) << ", " << peb_ref(2) << "\n";
}