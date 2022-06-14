#include "hopper_can_interface/can_interface.h"

#include <gtest/gtest.h>

TEST(can_interface, mockTest) {
  EXPECT_EQ(can_interface::helloWorld(), std::string("Hello"));
}