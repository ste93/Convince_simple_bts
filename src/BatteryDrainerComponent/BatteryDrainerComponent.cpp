/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/LogStream.h>

#include <BatteryDrainerService.h>
#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Property.h>
#include <yarp/dev/IBattery.h>
#include <yarp/os/RpcClient.h>

#include <mutex>
#include <set>

class BatteryDrainerComponent : public BatteryDrainerService
{
public:
    BatteryDrainerComponent() = default;

    bool open()
    {
        this->yarp().attachAsServer(server_port);
        if (!server_port.open("/BatteryDrainerComponent")) {
            yError("Could not open ");
            return false;
        }

        m_property_battery_nwc_yarp.put("device", "battery_nwc_yarp");
        m_property_battery_nwc_yarp.put("remote", "/battery_nws");
        m_property_battery_nwc_yarp.put("local", "/battery_drainer_nwc");

        // open ports

        if (!m_polyDriver_battery_nwc_yarp.open(m_property_battery_nwc_yarp)) {
            yWarning("Error! m_polyDriver_battery_nwc_yarp.open(m_property_battery_nwc_yarp)" );
            return false;
        }

        // attach services as clients

        if(!m_polyDriver_battery_nwc_yarp.view(m_ibattery)) {
            yWarning("Error! m_polyDriver_battery_nwc_yarp.view(m_ibattery)"  );
            return false;
        }

        if (!fakeBatteryRPC.open("/battery_drainer/fakeBatteryRPC")){
            yWarning("Error! fakeBatteryRPC.open /battery_drainer/fakeBatteryRPC"  );
            return false;
        }

        return true;
    }

    void close()
    {
        server_port.close();
    }

    void drain() override
    {
        m_ibattery->getBatteryCharge(m_level);
        yWarning() <<"draining" << m_level;    
        if (fakeBatteryRPC.getOutputCount()>0)
        {
            yarp::os::Bottle cmd;
            yarp::os::Bottle reply;
            cmd.addString("setBatteryCharge");
            yWarning() <<"setBatteryCharge" << m_level-5.0;
            cmd.addFloat64(m_level-5.0);
            yInfo() << "cmd" << cmd.toString();
            fakeBatteryRPC.write(cmd,reply);
            yInfo() << "reply" << reply.toString();
        }
    }

private:

    yarp::dev::PolyDriver m_polyDriver_battery_nwc_yarp;
    yarp::os::Property m_property_battery_nwc_yarp;
    yarp::dev::IBattery* m_ibattery{nullptr};
    yarp::os::RpcClient fakeBatteryRPC;

    double m_level { 100.0 };
    
    yarp::os::RpcServer server_port;
};

int main()
{
    yarp::os::Network yarp;

    BatteryDrainerComponent batteryDrainerComponent;
    if (!batteryDrainerComponent.open()) {
        return 1;
    }

    while (true) {
        yInfo("Server running happily");
        yarp::os::Time::delay(10);
    }

    batteryDrainerComponent.close();

    return 0;
}
