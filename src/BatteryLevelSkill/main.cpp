#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "BatteryLevelDataModel.h"
#include "BatteryLevelSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  BatteryLevelSkill stateMachine("BatteryLevel");
  stateMachine.start();

  int ret=app.exec();
  return ret;
  
}

