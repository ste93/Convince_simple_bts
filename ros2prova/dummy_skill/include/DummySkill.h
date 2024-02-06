/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <bt_interfaces/msg/request_ack.hpp>
#include <bt_interfaces/srv/request_ack.hpp>
#include <bt_interfaces/srv/send_start.hpp>
#include <bt_interfaces/srv/send_stop.hpp>
#include <mutex>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>

class DummySkill :
    public yarp::os::PortReader,
    public rclcpp::Node
{
public:
    DummySkill(std::string name );
    ~DummySkill();

    void request_ack( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::RequestAck::Request> request,
          std::shared_ptr<bt_interfaces::srv::RequestAck::Response>      response);
    void send_start( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::SendStart::Request> request,
                     [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::SendStart::Response>      response);
    void send_stop([[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::SendStop::Request> request,
                   [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::SendStop::Response>      response);
    bool read(yarp::os::ConnectionReader& connection) override;

private:
    uint8_t          m_status;
    std::string      m_name;
    std::mutex       m_mutex;
    yarp::os::Port   m_changeStatusPort;
    rclcpp::Service<bt_interfaces::srv::RequestAck>::SharedPtr m_requestAckService;
    rclcpp::Service<bt_interfaces::srv::SendStart>::SharedPtr  m_sendStartService;
    rclcpp::Service<bt_interfaces::srv::SendStop>::SharedPtr   m_sendStopService;

};
