/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "AlarmBatteryLowSkill.h"

#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

AlarmBatteryLowSkill::AlarmBatteryLowSkill(std::string name ) :
        m_name(std::move(name))
{
    //stateMachine.setDataModel(&dataModel);
}


void AlarmBatteryLowSkill::spin(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::spin(node);  
    rclcpp::shutdown();  
}


bool AlarmBatteryLowSkill::start()
{

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }

    m_node = rclcpp::Node::make_shared(m_name);
    m_requestAckService = m_node->create_service<bt_interfaces::srv::RequestAck>("AlarmBatteryLowRequestAck",  std::bind(&AlarmBatteryLowSkill::request_ack,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));
    m_sendStartService = m_node->create_service<bt_interfaces::srv::SendStart>("AlarmBatteryLowSendStart",  std::bind(&AlarmBatteryLowSkill::send_start,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));
                                                                                                                 
    m_sendStopService = m_node->create_service<bt_interfaces::srv::SendStop>("AlarmBatteryLowSendStop",  std::bind(&AlarmBatteryLowSkill::send_stop,
                                                                                                                 this,
                                                                                                                 std::placeholders::_1,
                                                                                                                 std::placeholders::_2));

    RCLCPP_DEBUG(m_node->get_logger(), "AlarmBatteryLowSkill::start");
    std::cout << "AlarmBatteryLowSkill::start";

    m_stateMachine.start();
    m_threadSpin = std::make_shared<std::thread>(spin, m_node);
    return true;
}

void AlarmBatteryLowSkill::request_ack( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::RequestAck::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::RequestAck::Response>      response)
{
    RCLCPP_DEBUG(m_node->get_logger(), "AlarmBatteryLowSkill::request_ack");
    std::cout << "AlarmBatteryLowSkill::request_ack";
    for (const auto& state : m_stateMachine.activeStateNames()) {
            std::cout << state.toStdString() << std::endl;
            auto message = bt_interfaces::msg::RequestAck();
            if (state == "alarm") {
                response->status.status = message.SKILL_RUNNING;
            }
            response->status.status = message.SKILL_IDLE;
    }
}

void AlarmBatteryLowSkill::send_start( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::SendStart::Request> request,
                                       [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::SendStart::Response>      response)
{
    RCLCPP_DEBUG(m_node->get_logger(), "AlarmBatteryLowSkill::send_start");    
    std::cout << "AlarmBatteryLowSkill::send_start";    
    m_stateMachine.submitEvent("CMD_START");
}

void AlarmBatteryLowSkill::send_stop( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::SendStop::Request> request,
                                       [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::SendStop::Response>      response)
{
    std::cout <<  "AlarmBatteryLowSkill::send_stop";    
    m_stateMachine.submitEvent("CMD_STOP",  QStateMachine::HighPriority);
}
