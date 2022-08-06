cmake_minimum_required(VERSION 3.0.2)
project(hopper_mpc)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(
  -std=c++17 
  -pthread
  -Wfatal-errors
  -Wl,--no-as-needed  
)

set(OpenGL_GL_PREFERENCE GLVND)

set(CATKIN_PACKAGE_DEPENDENCIES
  # hopper_can_interface
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

#find_package(mujoco REQUIRED)
find_package(raisim CONFIG REQUIRED)
#find_package(glfw3 REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
#  LIBRARIES hopper_mpc
 CATKIN_DEPENDS
   ${CATKIN_PACKAGE_DEPENDENCIES}
#  DEPENDS system_lib
)

# ###########
# ## Build ##
# ###########
include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/runner.cpp
  src/bridge_raisim.cpp
  src/leg.cpp 
  src/utils.cpp 
)
target_link_libraries(${PROJECT_NAME}
  #mujoco::mujoco
  #glfw
  raisim::raisim
  pthread
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    DISABLE_CLANG_TIDY
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############


#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
  test/test_hopper_mpc.cpp
)
target_link_libraries(${PROJECT_NAME}-test gtest_main)

