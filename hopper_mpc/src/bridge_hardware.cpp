#include "hopper_mpc/bridge_hardware.h"
#include <cstring>
#include <filesystem>
#include <fstream>  // for boost::serialization
#include <iostream>
#include "hopper_mpc/utils.hpp"

HardwareBridge::HardwareBridge(Model model_, double dt_, bool fixed_, bool skip_homing_) : Base(model_, dt_, fixed_, skip_homing_) {}

void HardwareBridge::Init() {
  // ROS subscribers
  int argcr = 0;
  char** argvr = NULL;
  ros::init(argcr, argvr, "hopper_ctrl");  // ROS

  mocapPtr.reset(new MocapSub(dt));

  cx5Ptr.reset(new Cx5());
  // i2c
  wt901Ptr.reset(new Wt901());

  // init variables
  qa.setZero();
  dqa.setZero();
  p.setZero();
  Q.setIdentity();  // is this correct?
  v.setZero();
  wb.setZero();
  p_prev.setZero();

  qla_home = model.qla_home;

  // ODrives
  ODriveCANleft.reset(new ODriveCan(Channel::CAN4, BandRate::BAUD_1M));
  node_id_q0 = 0;
  node_id_rwl = 3;

  ODriveCANright.reset(new ODriveCan(Channel::CAN3, BandRate::BAUD_1M));
  node_id_q2 = 1;
  node_id_rwr = 2;

  ODriveCANyaw.reset(new ODriveCan(Channel::CAN1, BandRate::BAUD_1M));
  node_id_rwz = 4;

  std::map<std::string, int> mapCANleft;
  mapCANleft.insert(std::make_pair("q0", node_id_q0));
  mapCANleft.insert(std::make_pair("rwl", node_id_rwl));
  std::map<std::string, int> mapCANright;
  mapCANright.insert(std::make_pair("q2", node_id_q2));
  mapCANright.insert(std::make_pair("rwr", node_id_rwr));
  std::map<std::string, int> mapCANyaw;
  mapCANyaw.insert(std::make_pair("rwz", node_id_rwz));

  ODriveCANleft->initialize(mapCANleft);
  ODriveCANright->initialize(mapCANright);
  ODriveCANyaw->initialize(mapCANyaw);

  float vel_lim_rmdx10 = 10;
  float cur_lim_rmdx10 = 60;  // 60;

  float vel_lim_r100 = 60;   // max 4400 rpm = 73 rps = 461 rad/s
  float cur_lim_r100 = 100;  // max 104

  float vel_lim_r80 = 60;  // max 5250 rpm = 87.5 rps = 550 rad/s
  float cur_lim_r80 = 46;  // max 46

  // NOTE: Always turn the power distribution on with the leg in the tight seated crouch position. EVEN WHEN HOMING WITH ROBOT FIXED IN
  // MIDAIR! Otherwise saved homing will be wrong. Because the ODrive # rotations is based on the pos you were at when it turned on
  if (skip_homing == false) {
    std::cout << "Robot WILL HOME! Make sure it is either in the jig or being held, then press any key to continue. \n";
    std::cin.ignore();
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));  // so I have time to run over to the testing jig
    std::cout << "Starting homing procedure. \n";
    Home(ODriveCANleft, node_id_q0, -1, cur_lim_rmdx10, vel_lim_rmdx10);
    Home(ODriveCANright, node_id_q2, -1, cur_lim_rmdx10, vel_lim_rmdx10);
    std::cout << "Finished homing procedure. \n";
    std::cout << "Saved homing offsets: q_offset = " << q_offset(0) << ", " << q_offset(1) << " \n";
    std::ofstream ofs("offsets.txt");  // create and open a character archive for output
    saved.reset(new saved_offsets(q_offset(0), q_offset(1)));
    {
      boost::archive::text_oarchive oa(ofs);  // save data to archive
      oa << *saved;                           // write class instance to archive
    }

    SetPosCtrlMode(ODriveCANleft, node_id_q0, model.q_init(0));
    SetPosCtrlMode(ODriveCANright, node_id_q2, model.q_init(2));
    std::cout << "Controller ready to begin. Press any key to continue. \n";
    std::cin.ignore();

    // SetPosCtrlMode(ODriveCANleft, node_id_q0, model.qla_sit(0));
    // SetPosCtrlMode(ODriveCANright, node_id_q2, model.qla_sit(1));

  } else {
    std::cout << "Robot will NOT home! Make sure it is in startup configuration, then press any key to continue. \n";
    std::cin.ignore();

    Startup(ODriveCANleft, node_id_q0, cur_lim_rmdx10, vel_lim_rmdx10);
    Startup(ODriveCANright, node_id_q2, cur_lim_rmdx10, vel_lim_rmdx10);
    saved_get.reset(new saved_offsets);
    // saved_offsets saved_get;
    {
      std::ifstream ifs("offsets.txt");  // create and open an archive for input
      boost::archive::text_iarchive ia(ifs);
      ia >> *saved_get;  // read class state from archive
    }
    q_offset(0) = saved_get->q0_offset;  // copy in
    q_offset(1) = saved_get->q2_offset;  // add # of rotations of rotor to get to starting configuration from homing position
    std::cout << "Loaded homing offsets: q_offset = " << q_offset(0) << ", " << q_offset(1) << " \n";
    SetPosCtrlMode(ODriveCANleft, node_id_q0, model.qla_sit(0));
    SetPosCtrlMode(ODriveCANright, node_id_q2, model.qla_sit(1));
    std::cout << "Once robot is in a stable sitting position, press any key to continue. \n";
    std::cin.ignore();
  }
  // initialize reaction wheels in torque control mode
  // DANGER!! disable while fiddling with IMU settings!!!
  Startup(ODriveCANright, node_id_rwr, cur_lim_r100, vel_lim_r100);
  Startup(ODriveCANleft, node_id_rwl, cur_lim_r100, vel_lim_r100);
  Startup(ODriveCANyaw, node_id_rwz, cur_lim_r80, vel_lim_r80);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void HardwareBridge::Home(std::unique_ptr<ODriveCan>& ODrive, int node_id, int dir, float cur_lim, float vel_lim) {
  ODrive->SetLimits(node_id, 2, 5);  // prevent motors from burning out when reaching end stops by reducing max current
  // Tell ODrive to find the leg position
  const double vel = 0.5;
  ODrive->SetControllerModes(node_id, ODriveCan::VELOCITY_CONTROL);
  ODrive->RunState(node_id, ODriveCan::AXIS_STATE_CLOSED_LOOP_CONTROL);
  ODrive->SetVelocity(node_id, vel * dir);
  float dq = vel;
  while (abs(dq) > 0.1) {
    // std::cout << "dq_measured = " << dq << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dq = ODrive->GetVelocity(node_id);
  }
  ODrive->SetVelocity(node_id, 0);                   // stop the motor so you can read position
  q_offset(node_id) = ODrive->GetPosition(node_id);  // read the RAW encoder position at home
  SetTorCtrlMode(ODrive, node_id);                   // switch to torque control so it is pliable!
  // ^ DON'T REMOVE THIS OR LEG WILL BLOCK ITSELF FROM FINISHING HOMING
  ODrive->SetLimits(node_id, cur_lim, vel_lim);  // set limits back to normal
  // ODrive->SetPositionGain(node_id, position_gain);
  // ODrive->SetVelocityGains(node_id, velocity_gain, velocity_integrator_gain);
}

void HardwareBridge::Startup(std::unique_ptr<ODriveCan>& ODrive, int node_id, float cur_lim, float vel_lim) {
  SetTorCtrlMode(ODrive, node_id);
  ODrive->SetLimits(node_id, vel_lim, cur_lim);
  ODrive->RunState(node_id, ODriveCan::AXIS_STATE_CLOSED_LOOP_CONTROL);
}

retVals HardwareBridge::SimRun(Eigen::Matrix<double, 5, 1> u, Eigen::Matrix<double, 2, 1> qla_ref, std::string ctrlMode) {
  // Torque vs Position Control for Legs
  // these would be set in constructor except that we may want to change mode in midst of control
  if (ctrlMode == "Pos") {
    if (ctrlMode_prev != "Pos") {
      SetPosCtrlMode(ODriveCANleft, node_id_q0, qla_ref(0));
      SetPosCtrlMode(ODriveCANright, node_id_q2, qla_ref(1));
    }
    SetJointPos(qla_ref);
  } else if (ctrlMode == "Torque") {
    if (ctrlMode_prev != "Torque") {
      SetTorCtrlMode(ODriveCANleft, node_id_q0);
      SetTorCtrlMode(ODriveCANright, node_id_q2);
    }
  }
  u(0) *= -1;
  u(2) *= -1;
  // u(4) *= -1;
  SetJointTorque(-u);  // needs to be outside the "if else" for reaction wheels
  qa = GetJointPos();
  dqa = GetJointVel();

  CheckEndStops(qa);
  // std::cout << qa(0) * 180 / M_PI << ", " << qa(1) * 180 / M_PI << ", ref = " << qla_ref(0) << ", " << qla_ref(1) << "\n";

  // --- begin collecting sensor data --- //
  ros::spinOnce();
  // get p and v from mocap
  p = mocapPtr->p;  // get position from mocap system
  v = (p - p_prev) / dt;

  // get Q, wb, and ab from cx5 IMU
  Q = cx5Ptr->Q;
  wb = cx5Ptr->omega;
  ab = cx5Ptr->alpha;

  // get aef from wt901 IMU
  // aef = wt901Ptr->CollectAcc();  // TODO: make this asynchronous otherwise it wastes too much time

  // get motor torques
  // tau = GetJointTorqueMeasured();  // TODO: Make this work
  tau = u;  // placeholder
  tau_ref = u;
  // --- end collecting sensor data --- //

  ctrlMode_prev = ctrlMode;
  p_prev = p;
  return retVals{p, Q, v, wb, ab, aef, qa, dqa};
}

void HardwareBridge::SetPosCtrlMode(std::unique_ptr<ODriveCan>& ODrive, int node_id, double q_init) {
  // set Odrives to position control
  Eigen::Vector2d qla_ref_init;
  qla_ref_init.setZero();
  qla_ref_init(node_id) = q_init;
  Eigen::Vector2d qo_ref = ConvertToODrivePos(qla_ref_init);
  ODrive->SetPosition(node_id, qo_ref(node_id));
  ODrive->SetControllerModes(node_id, ODriveCan::POSITION_CONTROL);
}

void HardwareBridge::SetTorCtrlMode(std::unique_ptr<ODriveCan>& ODrive, int node_id) {
  // set Odrives to torque control
  ODrive->SetTorque(node_id, 0);
  ODrive->SetControllerModes(node_id, ODriveCan::TORQUE_CONTROL);
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointPos() {
  Eigen::Matrix<double, 5, 1> qa;
  // gear ratio is 1/7, magnet is on motor which spins 7 times for every one output spin
  qa(0) = ODriveCANleft->GetPosition(node_id_q0);  // hardware joint facing opposite direction!!
  qa(1) = ODriveCANright->GetPosition(node_id_q2);
  qa += -q_offset;  // q0 and q2 should read 0 here if just homed

  // Conversion
  qa(0) *= -2 * M_PI / 7;
  //       ^ Negative sign is important!
  qa(1) *= 2 * M_PI / 7;
  qa(2) = ODriveCANright->GetPosition(node_id_rwr) * 2 * M_PI;
  qa(3) = ODriveCANleft->GetPosition(node_id_rwl) * 2 * M_PI;
  qa(4) = ODriveCANyaw->GetPosition(node_id_rwz) * 2 * M_PI;

  // correct homing position
  qa(0) += qla_home(0);
  qa(1) += qla_home(1);
  return qa;
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointVel() {
  Eigen::Matrix<double, 5, 1> dqa;
  dqa(0) = -ODriveCANleft->GetVelocity(node_id_q0) * 2 * M_PI / 7;  // ODrive velocity in RPS?
  dqa(1) = ODriveCANright->GetVelocity(node_id_q2) * 2 * M_PI / 7;
  dqa(2) = ODriveCANright->GetVelocity(node_id_rwr) * 2 * M_PI;
  dqa(3) = ODriveCANleft->GetVelocity(node_id_rwl) * 2 * M_PI;
  dqa(4) = ODriveCANyaw->GetVelocity(node_id_rwz) * 2 * M_PI;
  return dqa;
}

Eigen::Vector2d HardwareBridge::ConvertToODrivePos(Eigen::Vector2d qa) {
  // convert joint pos back to ODrive pos (turns instead of radians, no homed/offset, no gear ratio, motor polarity)
  Eigen::Vector2d qo;
  qo(0) = (-qla_home(0) + qa(0)) * -7 / (2 * M_PI) + q_offset(0);
  qo(1) = (-qla_home(1) + qa(1)) * 7 / (2 * M_PI) + q_offset(1);
  // qo(2) = qa(2) / (2 * M_PI);
  // qo(3) = qa(3) / (2 * M_PI);  // polarity?
  // qo(4) = qa(4) / (2 * M_PI);
  return qo;
}

Eigen::Vector2d HardwareBridge::ConvertToODriveVel(Eigen::Vector2d dqa) {
  // convert joint vel back to ODrive setup (turns instead of radians, no gear ratio, motor polarity)
  Eigen::Vector2d dqo;
  dqo(0) = -dqa(0) * 7 / (2 * M_PI);
  dqo(1) = dqa(1) * 7 / (2 * M_PI);
  // dqo(2) = dqa(2) / (2 * M_PI);
  // dqo(3) = dqa(3) / (2 * M_PI);  // polarity?
  // dqo(4) = dqa(4) / (2 * M_PI);
  return dqo;
}

void HardwareBridge::SetJointPos(Eigen::Vector2d qla_ref) {
  Eigen::Vector2d qo_ref = ConvertToODrivePos(qla_ref);
  ODriveCANleft->SetPosition(node_id_q0, qo_ref(0));
  ODriveCANright->SetPosition(node_id_q2, qo_ref(1));
}

void HardwareBridge::SetJointTorque(Eigen::Matrix<double, 5, 1> u) {
  ODriveCANleft->SetTorque(node_id_q0, -u(0));  // hardware joint facing opposite direction!!
  //                                   ^ Note the negative sign here.
  ODriveCANright->SetTorque(node_id_q2, u(1));
  ODriveCANright->SetTorque(node_id_rwr, u(2));
  ODriveCANleft->SetTorque(node_id_rwl, u(3));
  ODriveCANyaw->SetTorque(node_id_rwz, u(4));
}

Eigen::Matrix<double, 5, 1> HardwareBridge::GetJointTorqueMeasured() {
  Eigen::Matrix<double, 5, 1> tau;
  // TODO: Check if ODrive includes polarity in current measurement?
  tau(0) = -ODriveCANleft->GetIqMeasured(node_id_q0);  // ODrive velocity in RPS?
  tau(1) = ODriveCANright->GetIqMeasured(node_id_q2);
  tau(2) = -ODriveCANright->GetIqMeasured(node_id_rwr);
  tau(3) = ODriveCANleft->GetIqMeasured(node_id_rwl);
  tau(4) = ODriveCANyaw->GetIqMeasured(node_id_rwz);
  tau = tau.cwiseProduct(model.a_kt);  // multiply by actuator kt to convert current to torque
  return tau;
}

void HardwareBridge::CheckEndStops(Eigen::Matrix<double, 5, 1> qa) {
  // safety check to see if end stops are being hit, stop motors to prevent burnout
  if (qa(0) >= 28 * M_PI / 180 || qa(1) <= -185 * M_PI / 180) {
    End();
    stop == true;
    std::cout << "CheckEndStops: Joints hitting endstops, engaging e-stop \n";
  }
}

void HardwareBridge::End() {
  // disable the actuators, obviously
  ODriveCANleft->RunState(node_id_q0, ODriveCan::AXIS_STATE_IDLE);
  ODriveCANright->RunState(node_id_q2, ODriveCan::AXIS_STATE_IDLE);
  ODriveCANright->RunState(node_id_rwr, ODriveCan::AXIS_STATE_IDLE);
  ODriveCANleft->RunState(node_id_rwl, ODriveCan::AXIS_STATE_IDLE);
  ODriveCANyaw->RunState(node_id_rwz, ODriveCan::AXIS_STATE_IDLE);
}