// File: dust_remove.cpp
// Date: 24/03/2021
// Description: Supervisor controller to remove dust texture from simulation enviroment as robot drives over it. Shows where the robot has cleaned.
// Author: Bryan Higgins

#include <webots/Supervisor.hpp>
#include <webots/Display.hpp>

#define TIME_STEP 64

#define X 0
#define Y 1
#define Z 2

// Size of the floor. BH
#define GROUND_X 9.9
#define GROUND_Z 9.9


using namespace webots;

//NOTE: This controller is used on a robot node set as a supervisor in the scene tree. This is not a controller for the iRobot Create.
int main(int argc, char **argv) {
  
  //New supervisor instance
  Supervisor *supervisor = new Supervisor();
  //Gets display node from Webots scene tree. BH
  Display *display = supervisor->getDisplay("display");
  
  int width = display->getWidth();
  int height = display->getHeight();
  
  //Use supervisor to get the iRobot Create nodes details.
  Node *robotNode = supervisor->getFromDef("iRobotCreate");
  //Set dirt texture image. BH
  ImageRef *background = display->imageLoad("dirty.jpg");
  display->imagePaste(background, 0, 0, false);
  //Set pen for erasing dust texture. BH
  display->setAlpha(0.0);
  
  //Simulation while loop. BH
  while (supervisor->step(TIME_STEP) != -1) {
  //Get iRobot Creates current translation in the simulation enviroment. BH
  Field *translationField = robotNode->getField("translation");
  const double *translationValues = translationField->getSFVec3f();
  //Erase the dust texture where the robot has driven over, to show where the robot has cleaned. BH
  display->fillOval(width * (translationValues[X] + GROUND_X / 2) / GROUND_X, height * (translationValues[Z] + GROUND_Z / 2) / GROUND_Z, 7, 7);

  };

  //Clean up. BH
  delete supervisor;
  return 0;
}
