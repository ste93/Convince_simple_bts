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

    m_property_battery_nwc_yarp.put("device", "battery_nwc_yarp");
    m_property_battery_nwc_yarp.put("remote", "/battery_nws");
    m_property_battery_nwc_yarp.put("local", "/battery_drainer_nwc");

    // open ports

    if (!m_polyDriver_battery_nwc_yarp.open(m_property_battery_nwc_yarp)) {
        qWarning("Error! m_polyDriver_battery_nwc_yarp.open(m_property_battery_nwc_yarp)" );
        return false;
    }

    // attach services as clients

    if(!m_polyDriver_battery_nwc_yarp.view(m_ibattery)) {
        qWarning("Error! m_polyDriver_battery_nwc_yarp.view(m_ibattery)"  );
        return false;
    }

    if (!fakeBatteryRPC.open("/battery_drainer/fakeBatteryRPC")){
        qWarning("Error! fakeBatteryRPC.open /battery_drainer/fakeBatteryRPC"  );
        return false;
    }

    return true;
}
