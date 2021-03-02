// File: create_go_forward.cpp
// Date: 02/03/2021
// Description: A controller for the iRobot Create to kickstart our TSE project.
// Author: Bryan Higgins

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Receiver.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

int main(int argc, char **argv) {
  
  // Create the Robot instance. BH
  Robot *robot = new Robot();
  
  //Initializing distance sensors e.g. cliff sensors. BH
  DistanceSensor *ds[4];
  std::string dsNames[4] = {"cliff_left", "cliff_front_left", "cliff_front_right", "cliff_left"};
  for(int i = 0; i < 4; i++)
  {
   ds[i] = robot->getDistanceSensor(dsNames[i]);
   ds[i]->enable(TIME_STEP);
  }
  
  //Initializing bumpers e.g. touch sensors. BH
  TouchSensor *ts[2];
  std::string tsNames[2] = {"bumper_left", "bumper_right"}; 
  for(int i = 0; i < 2; i++)
  {
   ts[i] = robot->getTouchSensor(tsNames[i]);
   ts[i]->enable(TIME_STEP);
  }
  
  //Initializing position sensors. BH
  PositionSensor *ps[2];
  std::string psNames[2] = {"left wheel sensor", "right wheel sensor"};
  for(int i = 0; i < 2; i++)
  {
   ps[i] = robot->getPositionSensor(psNames[i]);
   ps[i]->enable(TIME_STEP);
  }
  
  //Initializing receiver. BH
  Receiver *r;
  std::string receiverName = "receiver";
  r = robot->getReceiver(receiverName);
  r->enable(TIME_STEP);
  
  //Initializing motors. BH
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // Main loop:
  //Avoidance algorithms go in here. BH
  while (robot->step(TIME_STEP) != -1)
  {
   //Read distance sensor outputs. BH
   double dsValues[4];
   for(int i = 0; i < 4; i++)
   {
    dsValues[i] = ds[i]->getValue();
   }
   
   //Read touch sensor outputs. BH
   double tsValues[2];
   for(int i = 0; i < 2; i++)
   {
    tsValues[i] = ts[i]->getValue();
   }
   
   //Read position sensor outputs. BH
   double psValues[2];
   for(int i = 0; i < 2; i++)
   {
    psValues[i] = ps[i]->getValue();
   }
   
   //Obstacle detection. BH
   bool left_collision = tsValues[0] > 0.0;
   bool right_collision = tsValues[1] > 0.0;
   bool is_there_vitual_wall = r->getQueueLength() > 0.0;
   bool cliff_left = dsValues[0] < 100.0 || dsValues[1] < 100.0;
   bool cliff_right = dsValues[3] < 100.0 || dsValues[2] < 100.0;
   bool cliff_front = dsValues[1] < 100.0 || dsValues[2] < 100.0;
  
  //Flush IR receiver. MUST STAY AT END OF LOOP. BH.
  while (r->getQueueLength() > 0){r->nextPacket();}
  delete robot;
  return 0;
  }
 }
