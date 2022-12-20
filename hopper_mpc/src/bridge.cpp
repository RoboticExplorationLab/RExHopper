#include "hopper_mpc/bridge.h"

Bridge::Bridge(Model model_, double dt_, std::string start_, bool skip_homing_) {
  // constructor
  model = model_;
  dt = dt_;
  g = model.g;
  mu = model.mu;
  start = start_;
  skip_homing = skip_homing_;
}