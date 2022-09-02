#include "hopper_mpc/bridge.h"

Bridge::Bridge(Model model_, double dt_, bool fixed_, bool record_) {
  // constructor
  model = model_;
  dt = dt_;
  g = model.g;
  mu = model.mu;
  fixed = fixed_;
  record = record_;
}