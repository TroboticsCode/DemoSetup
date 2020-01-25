#include "functions.h"
#include "DriveFunctionsConfig.h"
#include "vex.h"

void testPID()
{
  for(int i = 0; i < 4; i++)
  {
    moveLinear(24, 30);
    moveStop();
    moveRotate(90, 20);
    moveStop();
    moveLinear(24, 30);
    moveStop();
    moveRotate(90, 20);
    moveStop(); 
    moveLinear(24, 30);
    moveStop();
    moveRotate(90, 12);
    moveStop();
    moveLinear(24, 30);
    moveStop();
    moveRotate(90, 12);
    moveStop();
  }

    wait(20, msec); // Sleep the task for a short amount of time t
}