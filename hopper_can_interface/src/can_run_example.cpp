#include "hopper_can_interface/ODrivePCAN.h"

int main() {
  int BaudRate = 250000;
  ODrivePCAN odrivePcan(BaudRate);
  //odrivepcan::RunState(3, int requested_state);
  int l1 = 0;
  int l2 = 1;
  int rw1 = 2;
  int rw2 = 3;
  int rw3 = 4;
  float vel_target = 5;
  // odrivepcan.SetControllerModes(rw2, odrivepcan.ControlMode_t.VELOCITY_CONTROL, odrivepcan.InputMode_t.INACTIVE);
  odrivePcan.SetControllerModes(rw2, ODrivePCAN::ControlMode_t::VELOCITY_CONTROL, ODrivePCAN::InputMode_t::INACTIVE);
  odrivePcan.SetVelocity(rw2, vel_target);
  return 0;
}
