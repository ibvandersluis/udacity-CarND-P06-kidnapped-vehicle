cmake_minimum_required(VERSION 3.10)
project(udacity_particle_filter)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

include_directories(
  include
)

add_executable(particle_filter
  src/main.cpp
  src/particle_filter.cpp
)

target_link_libraries(particle_filter z ssl uv uWS)
