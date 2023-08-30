#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "StopAndWaitSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StopAndWaitSkill stateMachine("StopAndWait");
  stateMachine.start();

  int ret=app.exec();
  return ret;
  
}

