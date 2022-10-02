#include "hopper_can_interface/ODriveCan.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

using namespace hopper::can;

int main(int argc, char** argv) {
  ODriveCan ODrive(Channel::CAN3, BandRate::BAUD_1M);
  int node_id_q2 = 1;
  int node_id_rwr = 2;
  std::map<std::string, int> mapCANright;
  mapCANright.insert(std::make_pair("q2", node_id_q2));
  mapCANright.insert(std::make_pair("rwr", node_id_rwr));

  int node_id = node_id_rwr;
  // ODrive.initialize({{"test", node_id}});
  ODrive.initialize(mapCANright);
  double dir = 1;
  const double torque = 0.05;
  // ODrive.SetControllerModes(node_id, ODriveCan::TORQUE_CONTROL);
  // ODrive.RunState(node_id, ODriveCan::AXIS_STATE_CLOSED_LOOP_CONTROL);

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    dir = -dir;
    // ODrive.SetTorque(node_id, dir * torque);
    float pos = ODrive.GetPosition(node_id);
    std::cout << "Encoder pos = " << pos << "\n";
  }
}
