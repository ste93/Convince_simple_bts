/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

# pragma once

#include <Skill_request.h>

#include <yarp/os/RpcServer.h>
#include <yarp/os/Network.h>

#include "IsAtSkillStateMachine.h"
#include "IsAtSkillDataModel.h"

class IsAtSkill:
        public Skill_request
{
public:
    IsAtSkill(std::string name , const std::string location);

    bool start();

    SkillAck request_ack() override;
    void send_start() override;
    void send_stop() override;

private:
    std::string name;
    yarp::os::Network yarpnet;
    yarp::os::RpcServer port;
    IsAtSkillDataModel dataModel;
    IsAtSkillStateMachine stateMachine;
};
