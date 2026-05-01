cmake_minimum_required(VERSION 3.13)
project(pendarlab_lib_comm_mavlink_endpoint)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

find_package(GTest CONFIG REQUIRED)

add_library(pendarlab_comm_mavlink_endpoint SHARED
  src/MavlinkEndpoint.cpp
  src/MavlinkEndpointToken.cpp
)
target_include_directories(pendarlab_comm_mavlink_endpoint PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external> 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/byte_transport/include> 
)
target_compile_options(pendarlab_comm_mavlink_endpoint
  PRIVATE -Wno-address-of-packed-member
)