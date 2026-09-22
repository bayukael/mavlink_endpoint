cmake_minimum_required(VERSION 3.13)
project(pendarlab_lib_comm_mavlink_endpoint)

set(PROJECT_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
set(NAMESPACE pendarlab)
set(PACKAGE_FILENAME mavlink_endpoint)
set(PACKAGE_NAME PendarlabMavlinkEndpoint)

find_package(GTest CONFIG REQUIRED)
find_package(PendarlabByteTransport REQUIRED)

add_library(MavlinkEndpoint SHARED
  src/MavlinkEndpoint.cpp
  src/MavlinkEndpointToken.cpp
)
target_include_directories(MavlinkEndpoint PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external>
    $<INSTALL_INTERFACE:include> 
    $<INSTALL_INTERFACE:include/${PACKAGE_FILENAME}> 
)
target_compile_options(MavlinkEndpoint
  PRIVATE -Wno-address-of-packed-member
)
target_link_libraries(MavlinkEndpoint 
  PUBLIC pendarlab::ByteTransport
)
add_library(${NAMESPACE}::MavlinkEndpoint ALIAS MavlinkEndpoint)
set_target_properties(MavlinkEndpoint PROPERTIES OUTPUT_NAME pendarlab-mavlink_endpoint)


##############################################################################
######################     Define Files to Install     #######################
##############################################################################

set(TARGET_EXPORT_NAME MavlinkEndpointTargets)
set(TARGET_EXPORT_FILENAME mavlink_endpoint_targets)
set(PACKAGE_DIRECTORY_DESTINATION lib/cmake/${PACKAGE_NAME})

install(
	TARGETS MavlinkEndpoint
	EXPORT ${TARGET_EXPORT_NAME}
)
install(
	DIRECTORY include/
	DESTINATION include
)
install(
	DIRECTORY external/
	DESTINATION include/${PACKAGE_FILENAME}
)
install(
	EXPORT ${TARGET_EXPORT_NAME}
	FILE ${TARGET_EXPORT_FILENAME}.cmake
	NAMESPACE ${NAMESPACE}::
	DESTINATION ${PACKAGE_DIRECTORY_DESTINATION}
)

##############################################################################
##########################     CMake Packaging     ###########################
##############################################################################

include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PACKAGE_FILENAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
    INSTALL_DESTINATION ${PACKAGE_DIRECTORY_DESTINATION}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# -------------------------------------------------
# 5. Install Config + Version files
# -------------------------------------------------
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
    DESTINATION ${PACKAGE_DIRECTORY_DESTINATION}
)

##################################################################################################################
################################################  BUILDING TEST  #################################################
##################################################################################################################
add_executable(mavlink_endpoint_test test/mavlink_endpoint_test.cpp)
target_include_directories(mavlink_endpoint_test PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/mavlink/common> 
)
target_link_libraries(mavlink_endpoint_test
  PRIVATE MavlinkEndpoint
  PRIVATE gtest
  PRIVATE pendarlab::ByteTransport
)
target_compile_options(mavlink_endpoint_test
  PRIVATE -Wno-address-of-packed-member
)