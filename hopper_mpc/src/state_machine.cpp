#include "hopper_mpc/state_machine.h"
#include <iostream>
#include "hopper_mpc/tinyfsm.hpp"

class Fall;  // forward declaration

void StateMachine::react(Update const&) {
  std::cout << "Update event ignored" << std::endl;
}

void StateMachine::ReceiveData(bool s, bool sh, double dz) {
  s_ = s;
  sh_ = sh;
  dz_ = dz;
}

FSM_INITIAL_STATE(StateMachine, Fall)
