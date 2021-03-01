// File: create_go_forward.cpp
// Date: 22/02/2021
// Description: A basic controller for the iRobot Create to kickstart our TSE project.
// Author: Bryan Higgins

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <string>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

Robot *robot = new Robot();



TouchSensor *leftBumper = robot->getTouchSensor("bumper_left");
TouchSensor *rightBumper = robot->getTouchSensor("bumper_right");

Receiver *receiver = robot->getReceiver("receiver");
  
Motor *leftMotor = robot->getMotor("left wheel motor");
Motor *rightMotor = robot->getMotor("right wheel motor");
  


static void initialise(){
  std::cout << "Hi there" << std::endl;
  leftBumper->enable(1);
  rightBumper->enable(1);
  receiver->enable(1);
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(0.4*MAX_SPEED);
  rightMotor->setVelocity(0.4*MAX_SPEED);
}

static bool CollisionFront(){
  return (receiver->getQueueLength() > 0);
}

static bool CollisionLeft(){
  return (leftBumper->getValue() != 0.0);
}

static bool CollisionRight(){
  return(rightBumper->getValue() != 0.0);
}

static void flushIR(){
  while(receiver->getQueueLength() > 0){
    receiver->nextPacket();
  }
}



int main(int argc, char **argv) {
  
  // create the Robot instance.
  initialise();
  

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1){
    if(CollisionFront()){
      printf("Front receiver triggered\n");
    } else if(CollisionLeft()){
      printf("Left touch sensor triggered\n");
    } else if(CollisionRight()){
      printf("Right touch sensor triggered\n");
    }
    flushIR();
  }
  

  
  return 0;
}
