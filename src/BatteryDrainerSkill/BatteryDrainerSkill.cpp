/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "BatteryDrainerSkill.h"

#include <QTimer>
#include <QDebug>
#include <QtDebug>
#include <QTime>
#include <QStateMachine>

BatteryDrainerSkill::BatteryDrainerSkill(std::string name ) :
        name(std::move(name))
{
    stateMachine.setDataModel(&dataModel);
}

bool BatteryDrainerSkill::start()
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

SkillAck BatteryDrainerSkill::request_ack()
{
    while (true) {
        for (const auto& state : stateMachine.activeStateNames()) {


            if (state == "drain") {
                stateMachine.submitEvent("CMD_OK");
                return SKILL_RUNNING;
            }
            if (state == "idle") {
                stateMachine.submitEvent("CMD_OK");
                return SKILL_IDLE;

            }
        }
    }
}

void BatteryDrainerSkill::send_start()
{
    qWarning("sending start");
    stateMachine.submitEvent("CMD_START");
}

void BatteryDrainerSkill::send_stop()
{
    stateMachine.submitEvent("CMD_STOP",  QStateMachine::HighPriority);
}
