/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <QScxmlCppDataModel>
#include <QVariantMap>

#include <QTime>
#include <QTimer>
#include <QDebug>

#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>

#include "GoTo.h" 
 

struct Connector
{
    Connector(std::string from, std::string to, std::string carrier) :
            m_from(std::move(from)),
            m_to(std::move(to)),
            m_carrier(std::move(carrier))
    {
        if (!yarp::os::Network::connect(m_from, m_to, carrier)) {
            qFatal("Error! Could not connect to server %s", m_to.c_str());
        }
    }

    ~Connector()
    {
        if (!yarp::os::Network::disconnect(m_from, m_to)) {
            qFatal("Error! Could not disconnect from server %s", m_to.c_str());
        }
    }

    std::string m_from;
    std::string m_to;
    std::string m_carrier;
};

class IsAtSkillDataModel: public QScxmlCppDataModel
{
    Q_OBJECT
    Q_SCXML_DATAMODEL

public:
    IsAtSkillDataModel(const std::string location);

    bool setup(const QVariantMap& initialDataValues) override;

    yarp::os::Network yarp;

    yarp::os::RpcClient client_port_goTo;
    

    GoTo goTo;
    const std::string location;
    double squared_lin_distance { 100.0 };
    double squared_ang_distance { 100.0 };
    
};

Q_DECLARE_METATYPE(::IsAtSkillDataModel*)
