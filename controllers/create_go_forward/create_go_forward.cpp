// File: create_go_forward.cpp
// Date: 22/02/2021
// Description: A basic controller for the iRobot Create to kickstart our TSE project.
// Author: Bryan Higgins

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

int main(int argc, char **argv) {
  
  // create the Robot instance.
  Robot *robot = new Robot();
  
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(0.4*MAX_SPEED);
  rightMotor->setVelocity(0.4*MAX_SPEED);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1);

  delete robot;
  return 0;
}
