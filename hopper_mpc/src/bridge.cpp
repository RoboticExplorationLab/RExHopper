#include "hopper_mpc/bridge.h"

Bridge::Bridge(Model model_, double dt_, bool fixed_, bool home_) {
  // constructor
  model = model_;
  dt = dt_;
  g = model.g;
  mu = model.mu;
  fixed = fixed_;
  home = home_;
}