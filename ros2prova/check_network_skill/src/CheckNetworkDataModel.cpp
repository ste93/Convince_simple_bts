/******************************************************************************
 *                                                                            *
 * Copyright (C) 2024 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "CheckNetworkDataModel.h"

#include <QDebug>
#include <QTimer>
#include <QScxmlStateMachine>
#include <thread>

// Includes to perform ping
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;


void CheckNetworkDataModel::set_name(std::string name)
{
    m_name=name; 
}

bool CheckNetworkDataModel::setup(const QVariantMap &initialDataValues)
{
    Q_UNUSED(initialDataValues)

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    
    m_node = rclcpp::Node::make_shared(m_name);
    m_address_name = "127.0.0.1";

    timer_ = m_node->create_wall_timer(1000ms, std::bind(&CheckNetworkDataModel::topic_callback, this));

    RCLCPP_DEBUG(m_node->get_logger(), "CheckNetworkDataModel::start");
    m_thread = std::make_shared<std::thread>([this]{ start();});
    return true;
}


bool CheckNetworkDataModel::close()
{
    rclcpp::shutdown();
    return true;
}

void CheckNetworkDataModel::spin()
{
    rclcpp::spin(m_node);
}

bool CheckNetworkDataModel::start() {
    std::cout << "Before spinning" << std::endl;
    spin();  
    std::cout << "Finished spinning" << std::endl;
    close();
    return true;
}

void CheckNetworkDataModel::topic_callback() {
    std::cout << "CALLBACK" << std::endl;
    m_is_connected = isNetworkConnected(m_address_name);
    std::cout << "Our machine is connected? " << m_is_connected;
}

bool CheckNetworkDataModel::isNetworkConnected(const std::string& host) {

    struct addrinfo hints, *res;
    struct sockaddr_in r_addr;
    socklen_t r_addr_len;
    int sockfd;
    bool is_connected = false;

    // Setup hints
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_RAW;
    hints.ai_protocol = IPPROTO_ICMP;

    // Get address info -> this will take care of the dns lookup if needed
    if (getaddrinfo(host.c_str(), NULL, &hints, &res) != 0) {
        return false;
    }

    // Create socket
    // sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    sockfd = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (sockfd == -1) {
        std::cout << "Failed to create socket" << std::endl;
        freeaddrinfo(res);
        return false;
    }

    // Build a packet to send to host
    struct ping_pkt pckt;
    memset(&pckt,0,sizeof(pckt));
    pckt.hdr.type = ICMP_ECHO;
    pckt.hdr.un.echo.id = getpid();
    for (std::size_t i = 0; i<sizeof(pckt.msg) - 1; i++)
        pckt.msg[i] = i + '0';

    pckt.msg[sizeof(pckt.msg)-1] = 0;
    pckt.hdr.un.echo.sequence = m_msg_count++;
    pckt.hdr.checksum = checksum(& pckt, sizeof(pckt));

    // Send a packet to the host
    int bytes_sent = sendto(sockfd,&pckt,sizeof(pckt),0,res->ai_addr,res->ai_addrlen);
    if(bytes_sent == -1) {
        std::cout << "Failed to send packet address" << std::endl;
        freeaddrinfo(res);
        return false;
    }

    // Receive a packet
    char rbuffer[128];
    int bytes_received = recvfrom(sockfd,rbuffer,sizeof(rbuffer),0,
                                    (struct sockaddr*) &r_addr, &r_addr_len);
    if(bytes_received == -1) {
        std::cout << "Failed to receive packets" << std::endl;
        is_connected = false;
    }

    if(is_connected){
        std::cout << "is connected!" << std::endl;
    }

    freeaddrinfo(res);

    return is_connected;
}
