#include "hopper_mpc/bridge.h"

Bridge::Bridge(Model model, double dt, double g, double mu, bool fixed, bool record) {
  // constructor
  model_ = model;
  dt_ = dt;
  g_ = g;
  mu_ = mu;
  fixed_ = fixed;
  record_ = record;
}