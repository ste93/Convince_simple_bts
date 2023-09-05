################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

 # Create the component (navigation client)
add_executable(BatteryDrainerComponent)
target_sources(BatteryDrainerComponent
  PRIVATE
  BatteryDrainerComponent.cpp)
target_link_libraries(BatteryDrainerComponent
  PRIVATE
    BatteryDrainerService
    YARP::YARP_os
    YARP::YARP_init
    YARP::YARP_dev)
