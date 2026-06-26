cmake_minimum_required(VERSION 3.13)
project(pendarlab_lib_comm_mavlink_endpoint)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

find_package(GTest CONFIG REQUIRED)
find_package(PendarlabByteTransport REQUIRED)

add_library(MavlinkEndpoint SHARED
  src/MavlinkEndpoint.cpp
  src/MavlinkEndpointToken.cpp
)
target_include_directories(MavlinkEndpoint PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external>
)
target_compile_options(MavlinkEndpoint
  PRIVATE -Wno-address-of-packed-member
)
target_link_libraries(MavlinkEndpoint 
  PUBLIC pendarlab::ByteTransport
)
set_target_properties(MavlinkEndpoint PROPERTIES OUTPUT_NAME pendarlab-mavlink_endpoint)

##################################################################################################################
################################################  BUILDING TEST  #################################################
##################################################################################################################
add_executable(mavlink_endpoint_test test/mavlink_endpoint_test.cpp)
target_include_directories(mavlink_endpoint_test PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/mavlink/common> 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/byte_transport/include> 
)
target_link_libraries(mavlink_endpoint_test
  PRIVATE MavlinkEndpoint
  PRIVATE gtest
  PRIVATE pendarlab::ByteTransport
)
target_compile_options(mavlink_endpoint_test
  PRIVATE -Wno-address-of-packed-member
)