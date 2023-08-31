/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "IsAtSkillDataModel.h"
#include <QDebug>
#include <QTimer>
#include <QScxmlStateMachine>

IsAtSkillDataModel::IsAtSkillDataModel(const std::string location) :
                        location(std::move(location))
{
}

bool IsAtSkillDataModel::setup(const QVariantMap &initialDataValues)
{
    Q_UNUSED(initialDataValues)

    if (!yarp::os::NetworkBase::checkNetwork()) {
        qWarning("Error! YARP Network is not initialized");
        return false;
    }

    // open ports

    if (!client_port_goTo.open("/GoToIsAtClient/" + location)) {
        qWarning("Error! Cannot open YARP port with command: client_port_goTo.open(/GoToClient/location) " );
        return false;
    }

    // attach services as clients

    if(!goTo.yarp().attachAsClient(client_port_goTo)) {
        qWarning("Error! Could not attach as client with command : goTo.yarp().attachAsClient(client_port_goTo) "  );
        return false;
    }

    // open connections to components

    if (!yarp::os::Network::connect(client_port_goTo.getName(), "/GoToComponent", "tpP")) {
        qWarning("Error! Could not connect to server : /GoToComponent " );
        return false;
    }


    return true;
}
