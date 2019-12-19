#include "Functions.h"
#include "PID.h"

using namespace vex;

void pidInit(pidStruct_t* pid, double kP, double kD, int minDt)
{
  pid->kP = kP;
  pid->kD = kD;
  pid->minDt = minDt;

  pid->lastTime = Brain.timer(timeUnits::msec);
}

double pidCalculate(pidStruct_t *pid, double target, double current)
{
  pid->error = target - current;

  double dT = Brain.timer(timeUnits::msec) - pid->lastTime;
  if(dT < pid->minDt)
    return pid->lastOutput;

  pid->derivative = (pid->error - pid->lastError)/dT;

  pid->output = (pid->error * pid->kP) + (pid->derivative * pid->kD);

  if((pid->output) > 100)
  {
    if(pid->output > 0)
      pid->output = 100; //gets sign
    else if(pid->output == 0)
      pid->output = 0;
    else
      pid->output = -100;
  } 
  pid->lastError = pid->error;
  pid->lastTime = Brain.timer(timeUnits::msec);

  return pid->output;
}

void printPIDValues(pidStruct_t *pid)
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Error: %f\n", pid->error);
  Brain.Screen.newLine();
  Brain.Screen.print("derivative: %f\n", pid->derivative);
  Brain.Screen.newLine();
  Brain.Screen.print("output: %f", pid->output);

  wait(20, msec);
}