/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <set>

class FakeBattery
{
public:
    FakeBattery() = default;

    bool open()
    {
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);

        m_node = rclcpp::Node::make_shared("FakeBatteryNode", node_options);
        m_drainService = m_node->create_service<sensor_msgs::srv::BatteryState>("FakeBattery/setCharge",  
                                                                                    std::bind(&FakeBattery::setCharge,
                                                                                    this,
                                                                                    std::placeholders::_1,
                                                                                    std::placeholders::_2));
        if (m_node == nullptr) {
            yCError(BATTERY_NWS_ROS2) << " opening " << m_nodeName << " Node, check your yarp-ROS2 network configuration\n";
            return false;
        }
        m_ros2Publisher = m_node->create_publisher<sensor_msgs::msg::BatteryState>("FakeBattery/state", 10);
        return true;
    }

    bool close()
    {
        rclcpp::shutdown();  
        return true;
    }

    void spin()
    {
        rclcpp::spin(m_node);  
    }


    void start() {
        spin();
        close();
    }


    void setCharge([[maybe_unused]] const std::shared_ptr<sensor_msgs::srv::BatteryState::Request> request,
               [[maybe_unused]] std::shared_ptr<sensor_msgs::srv::BatteryState>      response) 
    {
        m_level = request->percentage;
        std::cout <<"set charge" << request->percentage;  
    }

    void publishStatus() 
    {
        battMsg.voltage = 5;
        battMsg.current = 1;
        battMsg.temperature = 50;
        battMsg.charge = std::numeric_limits<double>::quiet_NaN();//std::nan("");
     //   battMsg.capacity = std::nan("");
      //  battMsg.design_capacity = std::nan("");
        battMsg.percentage = m_level;
        battMsg.power_supply_status = 0;
        battMsg.power_supply_health = 0;
        battMsg.power_supply_technology = 0;
        battMsg.present=true;
        battMsg.location="";
        battMsg.serial_number="";

        battMsg.header.frame_id = "" ;
        battMsg.header.stamp = m_node->get_clock()->now();

        m_ros2Publisher->publish(battMsg);
        std::cout << "charge" << m_level;
    }

    void discharge(double percentage) 
    {
        m_level = m_level - percentage;
    }

private:
    double m_level { 100.0 };
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Service<bt_interfaces::srv::RpcWithoutParameters>::SharedPtr m_drainService;

};
