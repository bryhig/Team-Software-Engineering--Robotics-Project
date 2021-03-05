// File: vacuum_cleaning_robot_controller.cpp
// Date: 03/03/2021
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
#define MAX_SPEED 6.0
#define HALF_SPEED 3.0
#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756

using namespace webots;





//Obstacle detection functions that check to see if sensors are at threshold value. BH
//collision and cliff functions work for all directions. Change function inputs for different directions. BH
bool collision(double tsValue)
{
 bool collision = tsValue > 0.0;
 return collision;
}

bool cliff(double dsValue1, double dsValue2)
{
 bool cliff = dsValue1 < 100.0 || dsValue2 < 100.0;
 return cliff;
}


// Not in use now, will be usable after code is restructured.

void go_forward(Motor *left, Motor *right, double speed){
  left->setVelocity(speed);
  right->setVelocity(speed);
}

void go_backward(Motor *left, Motor *right, double speed){
  left->setVelocity(-speed);
  right->setVelocity(-speed);
}

void stop(Motor *left, Motor *right){
  left->setVelocity(0.0);
  right->setVelocity(0.0);
}

// Partially pulled from the webots robot, rebuilt for C++. DC
// Needs step functions in order to not crash webots. DC

void turn(Motor *left, Motor *right, PositionSensor *posSensors[2], double angle){
  // Grab the current positions of the wheels (at start of turning) to apply when working out how far we've turned. DC
  double leftOffset = posSensors[0]->getValue();
  double rightOffset = posSensors[1]->getValue();
  std::cout << posSensors[0]->getValue() << std::endl;
  
  // If the angle is negative, turn left, else turn right. DC
  double direction = (angle < 0) ? -1.0 : 1.0;
  std::cout << "works1" << std::endl;
  // Set the motors going - if left, set the left motor forward and right backward,
  // otherwise do the opposite. DC
  left->setVelocity(direction * HALF_SPEED);
  right->setVelocity(-direction * HALF_SPEED);
  std::cout << "works2" << std::endl;
  
  //until this variable == the angle entered, keep rotating - DC
  double orientation;
  do {
    // Grab the current positions of the wheels, DC
    double l = posSensors[0]->getValue() - leftOffset;
    double r = posSensors[1]->getValue() - rightOffset;
    std::cout << "worksL1" << std::endl;
    // Work out the distance we move based on the radius of the wheels, DC
    double dl = l * WHEEL_RADIUS;
    double dr = r * WHEEL_RADIUS;
    std::cout << "worksL2" << std::endl;
    // work out the current orientation by shifting the circle by the distance moved. DC
    // Circle diameter = axle length. DC
    orientation = direction * (dl-dr) * AXLE_LENGTH; 
  } while(orientation < direction * angle);
}


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
   
   // Read the position sensors. DC
   printf("%lf on the left, and %lf on the right\n", ps[0]->getValue(), ps[1]->getValue());
   
   // Tests for each movement function. DC
   //go_forward(leftMotor, rightMotor, MAX_SPEED);
   go_backward(leftMotor, rightMotor, HALF_SPEED);
   //turn(leftMotor, rightMotor, ps, 90);
   
   
   // Obstacle detection functions called. Input parameters specify direction for collision and cliff. BH
   bool isLeftCollision = collision(tsValues[0]);
   bool isRightCollision = collision(tsValues[1]);
   bool isThereWall = r->getQueueLength() > 0.0;
   bool isCliffLeft = cliff(dsValues[0], dsValues[1]);
   bool isCliffRight = cliff(dsValues[3], dsValues[2]);
   bool isCliffFront = cliff(dsValues[1], dsValues[2]);
  
   // Flush IR receiver. MUST STAY AT END OF LOOP. BH.
   while (r->getQueueLength() > 0){r->nextPacket();}
   
   
  }
  // Moved these here as they were killing the loop after 1 pass. DC
  delete robot;
  return 0;
 }
