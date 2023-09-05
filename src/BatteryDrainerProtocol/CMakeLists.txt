################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the BatteryDrainerService C++ Library
add_library(BatteryDrainerService STATIC)
yarp_add_idl(IDL_GEN_FILES BatteryDrainerService.thrift)
target_sources(BatteryDrainerService
  PRIVATE
    ${IDL_GEN_FILES})
target_include_directories(BatteryDrainerService
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(BatteryDrainerService
  PRIVATE
    YARP::YARP_os)
