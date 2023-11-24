/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <other_interfaces/srv/rpc_without_parameters.hpp>
#include <set>

class BatteryDrainerComponent
{
public:
    BatteryDrainerComponent() = default;

    bool open()
    {
        m_node = rclcpp::Node::make_shared("BatteryDrainerComponentNode");
        m_drainService = m_node->create_service<other_interfaces::srv::RpcWithoutParameters>("BatteryDrainerComponent/drain",  
                                                                                    std::bind(&BatteryDrainerComponent::drain,
                                                                                    this,
                                                                                    std::placeholders::_1,
                                                                                    std::placeholders::_2));
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

    void drain([[maybe_unused]] const std::shared_ptr<other_interfaces::srv::RpcWithoutParameters::Request> request,
               [[maybe_unused]] std::shared_ptr<other_interfaces::srv::RpcWithoutParameters::Response>      response) 
    {
        //m_ibattery->getBatteryCharge(m_level);
        
        std::cout <<"draining" << m_level;  

    }

private:
    double m_level { 100.0 };
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Service<other_interfaces::srv::RpcWithoutParameters>::SharedPtr m_drainService;

};
