#include "hopper_can_interface/can_interface.h"

#include <gtest/gtest.h>

TEST(can_interface, mockTest) {
  EXPECT_EQ(can_interface::helloWorld(), std::string("Hello"));  // arguments 1 and 2 must be equal
}