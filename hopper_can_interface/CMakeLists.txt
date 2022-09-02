cmake_minimum_required(VERSION 3.1.0)
project(hopper_can_interface)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wfatal-errors")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors")

# Pthread
set(CMAKE_THREAD_PREFER_PTHREAD ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
)

# ##########
# # Build ##
# ##########
include_directories(
  include
  /usr/include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/BufferedCanMsg.cpp
  src/CanInterface.cpp
  src/ODriveCan.cpp
)
target_link_libraries(${PROJECT_NAME} pcanbasic Threads::Threads)

# ############
# # Install ##
# ############

# ############
# # Testing ##
# ############
if(${CATKIN_ENABLE_TESTING})
  add_executable(TestCanWrite
    test/TestCanWrite.cpp
  )
  target_link_libraries(TestCanWrite ${PROJECT_NAME})

  add_executable(TestCanRead
    test/TestCanRead.cpp
  )
  target_link_libraries(TestCanRead ${PROJECT_NAME})

  add_executable(TestODrive
    test/TestODrive.cpp
  )
  target_link_libraries(TestODrive ${PROJECT_NAME})
endif()
