/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/


enum GoToStatus
{
    NOT_STARTED,
    RUNNING,
    SUCCESS,
    ABORT
}

struct RobotPose
{
double x;
double y;
double theta;
}

service GoTo {
  void goTo(1: string destination);
  GoToStatus getStatus(1: string destination);
  void halt(1: string destination);
  double isAtLocation (1: string destination);
  RobotPose getCurrentPose ();
  RobotPose getPoseOf (1: string destination);
  RobotPose getCurrentNavGoal ();

}
