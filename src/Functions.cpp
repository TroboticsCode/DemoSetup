#include "functions.h"
#include "DriveFunctionsConfig.h"
#include "vex.h"

void testPID()
{
  for(int i = 2; i < 4; i++)
  {
    //moveLinear(12, 100);  
    moveRotate(90, 50);

  }
  //moveStop();
  wait(20, msec); // Sleep the task for a short amount of time t
}