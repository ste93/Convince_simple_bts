################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the GoToProtocol C++ Library
add_library(GoToProtocol STATIC)
yarp_add_idl(IDL_GEN_FILES GoTo.thrift)
target_sources(GoToProtocol
  PRIVATE
    GoTo.cpp
    GoTo.h
    GoToStatus.cpp
    GoToStatus.h
    RobotPose.cpp
    RobotPose.h
)
target_include_directories(GoToProtocol
   INTERFACE
     $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(GoToProtocol
  PRIVATE
    YARP::YARP_os)


