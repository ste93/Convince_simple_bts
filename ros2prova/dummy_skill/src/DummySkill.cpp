/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "DummySkill.h"

#include <iostream>

DummySkill::DummySkill(std::string name ) : Node(name)
{
    m_name = name;
    std::string portName = "/"+m_name+"/rpc:i";
    if(!m_changeStatusPort.open(portName.c_str()))
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open rpc port");
    }
    m_changeStatusPort.setReader(*this);

    RCLCPP_DEBUG(this->get_logger(), "%s::start",m_name.c_str());
    std::cout << m_name << "::start\n";
    std::string reqAckName = "/"+m_name+"/RequestAck";
    std::string sendStartName = "/"+m_name+"/SendStart";
    std::string sendStopName = "/"+m_name+"/SendStop";
    m_requestAckService = this->create_service<bt_interfaces::srv::RequestAck>(reqAckName,
                                                                                std::bind(&DummySkill::request_ack,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_sendStartService = this->create_service<bt_interfaces::srv::SendStart>(sendStartName,
                                                                                std::bind(&DummySkill::send_start,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));
    m_sendStopService = this->create_service<bt_interfaces::srv::SendStop>(sendStopName,
                                                                                std::bind(&DummySkill::send_stop,
                                                                                this,
                                                                                std::placeholders::_1,
                                                                                std::placeholders::_2));

}

void DummySkill::request_ack( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::RequestAck::Request> request,
                                       std::shared_ptr<bt_interfaces::srv::RequestAck::Response>      response)
{
    RCLCPP_DEBUG(this->get_logger(), "%s::request_ack",m_name.c_str());
    std::cout << m_name <<  "::request_ack\n";

    response->status.status = m_status;
    response->is_ok = true;
}
void DummySkill::send_start( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::SendStart::Request> request,
                                       [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::SendStart::Response>      response)
{
    RCLCPP_DEBUG(this->get_logger(), "%s::send_start",m_name.c_str());
    std::cout << m_name <<  "::send_start\n";
    auto message = bt_interfaces::msg::RequestAck();
    m_status = message.SKILL_SUCCESS;
    response->is_ok = true;
}

void DummySkill::send_stop( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::SendStop::Request> request,
                                       [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::SendStop::Response>      response)
{

    RCLCPP_DEBUG(this->get_logger(), "%s::send_stop",m_name.c_str());
    std::cout << m_name <<  "::send_stop\n";
    auto message = bt_interfaces::msg::RequestAck();
    m_status = message.SKILL_IDLE;
    response->is_ok = true;
}

bool DummySkill::read(yarp::os::ConnectionReader& connection)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok = in.read(connection);
    if (!ok) return false;

    //parse string command
    if(in.get(0).isString())
    {
        std::string cmd = in.get(0).asString();
        auto message = bt_interfaces::msg::RequestAck();
        if(cmd=="help")
        {
            out.addVocab32("many");
            out.addString("RPC commands for Dummy skills\n");
            out.addString("\t setSuccess - Sets the current status to success\n");
            out.addString("\t setRunning - Sets the current status to running\n");
            out.addString("\t setFailure - Sets the current status to failure\n");
            out.addString("\t setIdle - Sets the current status to idle\n");
            out.addString("\t getStatus - Gets the current status\n");
            out.addString("");
        }
        else if(cmd=="setSuccess")
        {
            m_status = message.SKILL_SUCCESS;
            out.addString("ack");
        }
        else if(cmd=="setRunning")
        {
            m_status = message.SKILL_RUNNING;
            out.addString("ack");
        }
        else if(cmd=="setFailure")
        {
            m_status = message.SKILL_FAILURE;
            out.addString("ack");
        }
        else if(cmd=="getStatus")
        {
            switch (m_status)
            {
            case message.SKILL_IDLE:
                out.addString("IDLE");
                break;
            case message.SKILL_RUNNING:
                out.addString("RUNNING");
                break;
            case message.SKILL_SUCCESS:
                out.addString("SUCCESS");
                break;
            case message.SKILL_FAILURE:
                out.addString("FAILURE");
                break;

            default:
                break;
            }
        }
        else if(cmd=="setIdle")
        {
            m_status = message.SKILL_IDLE;
            out.addString("ack");
        }
    }
    else{
        out.addString("nack");
        return false;
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        out.write(*returnToSender);
    }


    return true;
}

DummySkill::~DummySkill()
{
    if(m_changeStatusPort.isOpen())
    {
        std::cout << "Closing ports...\n";
        RCLCPP_DEBUG(this->get_logger(), "Closing ports...");
        m_changeStatusPort.close();
    }
}
