cmake_minimum_required(VERSION 3.22)

set (project-name AlarmBatteryLowSkill)
project (${project-name})

#find_package(YARP COMPONENTS os REQUIRED)
#find_package(Qt6 COMPONENTS Scxml Core REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# set up our program

add_executable(${project-name} )
# declare our source files
file(GLOB_RECURSE COMMON_SOURCE "*.h" "*.cpp")


target_link_libraries(${project-name} PRIVATE ${YARP_LIBRARIES} Qt6::Core Qt6::Scxml Qt6::StateMachine BT_protocol)

target_sources(${project-name} PUBLIC main.cpp AlarmBatteryLowSkill.cpp AlarmBatteryLowSkill.h)

qt6_add_statecharts(${project-name} AlarmBatteryLowSM.scxml)
