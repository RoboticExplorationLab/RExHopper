cmake_minimum_required(VERSION 3.0.2)
project(hopper_can_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wfatal-errors")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors")

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

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

add_library(${PROJECT_NAME}
  src/BufferedCanMsg.cpp
  src/CanInterface.cpp
)
target_link_libraries(${PROJECT_NAME} pcanbasic Threads::Threads)

add_executable(TestCanWrite
  test/TestCanWrite.cpp
)
target_link_libraries(TestCanWrite ${PROJECT_NAME})

add_executable(TestCanRead
  test/TestCanRead.cpp
)
target_link_libraries(TestCanRead ${PROJECT_NAME})

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

