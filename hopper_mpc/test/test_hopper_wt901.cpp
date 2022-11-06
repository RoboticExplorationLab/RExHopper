#include <gtest/gtest.h>
#include <iostream>
#include "hopper_mpc/wt901.h"

TEST(AAA, test) {
  Wt901 wt901;
  wt901Vals wt901vals = wt901.Collect();

  std::cerr << wt901vals.acc << std::endl;
}