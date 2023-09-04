#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "BatteryDrainerDataModel.h"
#include "BatteryDrainerSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  BatteryDrainerSkill batteryDrainerSkill("BatteryDrainer");
  batteryDrainerSkill.start();

  int ret=app.exec();
  return ret;
  
}

