/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "StopAndWaitSkill.h"

#include <QTimer>
#include <QDebug>
#include <QTime>
#include <QStateMachine>

StopAndWaitSkill::StopAndWaitSkill(std::string name ) :
        name(std::move(name))
{
    //stateMachine.setDataModel(&dataModel);
}

bool StopAndWaitSkill::start()
{
    if (!yarp::os::NetworkBase::checkNetwork()) {
        qWarning("Error! YARP Network is not initialized");
        return false;
    }

    if (!port.open("/" + name + "/BT_rpc/server")) {
        qWarning("Error! Cannot open YARP port");
        return false;
    }

    if(!this->yarp().attachAsServer(port)) {
        qWarning("Error! Could not attach as server");
        return false;
    }

    stateMachine.start();

    return true;
}

SkillAck StopAndWaitSkill::request_ack()
{
    while (true) {
        for (const auto& state : stateMachine.activeStateNames()) {
            if (state == "idle") {
                stateMachine.submitEvent("CMD_OK");
                return SKILL_IDLE;
            }
            if (state == "wait") {
                stateMachine.submitEvent("CMD_OK");
                return SKILL_RUNNING;
            }
        }
    }
}

void StopAndWaitSkill::send_start()
{
    stateMachine.submitEvent("CMD_START");
}

void StopAndWaitSkill::send_stop()
{
    stateMachine.submitEvent("CMD_STOP",  QStateMachine::HighPriority);
}
