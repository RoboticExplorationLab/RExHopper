cmake_minimum_required(VERSION 3.0.2)
project(hopper_can_interface)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/can_interface.cpp
  src/ODrivePCAN.cpp
)

add_executable(test_odrive
  src/can_run_example.cpp
)

#############
## Install ##
#############


#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
  test/test_hopper_can_interface.cpp
)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} gtest_main)

