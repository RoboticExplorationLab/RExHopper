#include "hopper_can_interface/ODriveCan.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

using namespace hopper::can;

int main(int argc, char** argv) {
  ODriveCan ODrive(Channel::CAN3, BandRate::BAUD_1M);
  ODrive.initialize();
  double dir = 1;
  const double torque = 0.05;
  ODrive.SetControllerModes(0, ODriveCan::TORQUE_CONTROL);
  ODrive.RunState(0, ODriveCan::AXIS_STATE_CLOSED_LOOP_CONTROL);

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    dir = -dir;
    ODrive.SetTorque(0, dir * torque);
  }
}
