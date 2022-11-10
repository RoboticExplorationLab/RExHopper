#include <gtest/gtest.h>
#include <iostream>
#include "hopper_mpc/cx5.h"

TEST(BBB, test) {
  // reminder: run the docker image first
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "hopper_ctrl");
  ros::AsyncSpinner spinner(0);  // run threads async
  spinner.start();
  ros::waitForShutdown();
  Cx5 cx5;

  for (int i = 0; i < 1000; i++) {
    std::cout << cx5.Q.coeffs().transpose() << "\n";
  }
}