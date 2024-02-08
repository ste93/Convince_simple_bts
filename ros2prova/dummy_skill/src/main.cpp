#include <iostream>
#include "DummySkill.h"
#include <yarp/os/ResourceFinder.h>

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
    yarp::os::Network yarp(yarp::os::YARP_CLOCK_SYSTEM);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    if(!rf.check("skill_name"))
    {
        std::cout << "ERROR: Missing skill_name parameter\n";
        RCLCPP_ERROR(rclcpp::get_logger("DummySkillLogger"), "Missing skill_name parameter");
        return 1;
    }
    rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    std::shared_ptr<DummySkill> dummySkill = std::make_shared<DummySkill>(rf.find("skill_name").asString());
    rclcpp::spin(dummySkill);
    rclcpp::shutdown();

    return 0;

}

