cmake_minimum_required(VERSION 3.10.1)

include(FetchContent)
FetchCOntent_Declare(yaml-cpp 
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
)

FetchContent_MakeAvailable(yaml-cpp)

set (CMAKE_CXX_STANDARD 17)
project(RExHopper)
include_directories(include)

add_executable(main src/main.cpp)