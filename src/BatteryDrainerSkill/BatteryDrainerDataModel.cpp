/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "BatteryDrainerDataModel.h"

#include "BatteryDrainerDataModel.h"
#include <QDebug>
#include <QTimer>
#include <QScxmlStateMachine>



bool BatteryDrainerDataModel::setup(const QVariantMap &initialDataValues)
{
    Q_UNUSED(initialDataValues)

    if (!yarp::os::NetworkBase::checkNetwork()) {
        qWarning("Error! YARP Network is not initialized");
        return false;
    }

    if (!client_port_batteryDrainerService.open("/BatteryDrainerSkillRpc:o")) {
        qWarning("Error! Cannot open YARP port with command: client_port_batteryDrainerService.open(/BatteryDrainerSkillRpc:o) " );
        return false;
    }

    // attach services as clients

    if(!batteryDrainerService.yarp().attachAsClient(client_port_batteryDrainerService)) {
        qWarning("Error! Could not attach as client with command : batteryDrainerService.yarp().attachAsClient(client_port_batteryDrainerService) "  );
        return false;
    }


    // open connections to components

    if (!yarp::os::Network::connect(client_port_batteryDrainerService.getName(), "/BatteryDrainerComponent", "tcp")) {
        qWarning("Error! Could not connect to server : /BatteryComponent " );
        return false;
    }



    return true;
}
