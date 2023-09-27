#include "vex.h"
#include "Autons.h"
#include "Functions.h"
#include "DriveFunctionsConfig.h"

autonsTuple autons[] = {{"AUTON_1", (int*)Auton1}};
//Put your auton routines in here

void Auton1()
{
  //setRotGains(0, 0, 0, 20, 10); //update PID gains to tune robot
  //setLinGains(0, 0, 0, 20, 10);

  moveLinear(24, 100, 10000);
}