################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the skill (state machine executable)
cmake_minimum_required(VERSION 3.22)

set (project-name IsAtSkill)
project (${project-name})
add_executable(${project-name})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
file(GLOB_RECURSE COMMON_SOURCE "*.h" "*.cpp")
target_link_libraries(${project-name} PRIVATE ${YARP_LIBRARIES} Qt6::Core Qt6::Scxml Qt6::StateMachine BT_protocol GoToProtocol)

target_sources(${project-name} PUBLIC main.cpp IsAtSkillDataModel.h IsAtSkillDataModel.cpp IsAtSkill.cpp IsAtSkill.h)

qt6_add_statecharts(${project-name} IsAtSkillStateMachine.scxml)
