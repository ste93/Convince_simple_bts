/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "BatteryLevelSkill.h"

#include <QTimer>
#include <QDebug>
#include <QTime>
#include <QStateMachine>

BatteryLevelSkill::BatteryLevelSkill(std::string name ) :
        name(std::move(name))
{
    stateMachine.setDataModel(&dataModel);
}

bool BatteryLevelSkill::start()
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

SkillAck BatteryLevelSkill::request_ack()
{
    while (true) {
        for (const auto& state : stateMachine.activeStateNames()) {
            if (state == "idle") {
                stateMachine.submitEvent("CMD_OK");
                return SKILL_IDLE;
            }
            if (state == "get") {
                stateMachine.submitEvent("CMD_OK");
                return SKILL_IDLE;
            }
            if (state == "success") {
                stateMachine.submitEvent("CMD_OK");
                return SKILL_SUCCESS;
            }
            if (state == "failure") {
                stateMachine.submitEvent("CMD_OK");
                return SKILL_FAILURE;
            }

        }
    }
}

void BatteryLevelSkill::send_start()
{
    stateMachine.submitEvent("CMD_START");
}

void BatteryLevelSkill::send_stop()
{
    stateMachine.submitEvent("CMD_STOP",  QStateMachine::HighPriority);
}
