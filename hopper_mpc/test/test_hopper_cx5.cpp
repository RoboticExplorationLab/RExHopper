#include <gtest/gtest.h>
#include <iostream>
#include "hopper_mpc/cx5.h"

TEST(AAA, test) {
  Cx5 cx5;
  cx5Vals cx5vals = cx5.Collect();

  std::cerr << cx5vals.Q << std::endl;
}