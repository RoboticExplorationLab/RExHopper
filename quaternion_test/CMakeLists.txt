cmake_minimum_required(VERSION 3.0.2)
project(quaternion_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(
  -std=c++17 
  -pthread
  -Wfatal-errors
  -Wl,--no-as-needed  
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)
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

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}  # // remove?
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/quaternion_test.cpp
# )


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/pub_quat.cpp
  src/sub_cx5.cpp
  src/utils.cpp
  )


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#########################
###   CLANG TOOLING   ###
#########################

# this causes a warning for some reason

# "[cmake_clang_tools] Target quaternion_test does not exists.  Not adding
#  dependency."

# find_package(cmake_clang_tools QUIET)
# if (cmake_clang_tools_FOUND)
#   message(STATUS "Run clang tooling")
#   add_clang_tooling(
#     TARGETS ${PROJECT_NAME}
#     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
#     DISABLE_CLANG_TIDY
#     CF_WERROR
#   )
# endif (cmake_clang_tools_FOUND)