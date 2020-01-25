#include "Functions.h"
#include "PID.h"

using namespace vex;

void pidInit(pidStruct_t* pid, double kP, double kI, double kD, double slewRate, int minDt)
{
  pid->kP = kP;
  pid->kI = kI;
  pid->kD = kD;
  pid->slewRate = slewRate;
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
  pid->integral += (pid->error * dT);

  pid->output = (pid->error * pid->kP) + (pid->integral * pid->kI) + (pid->derivative * pid->kD);

//constrain output
  if(pid->output > 1)
    pid->output = 1; 
  if(pid->output < -1)
    pid->output = -1;

//slewing
  if((pid->output > pid->lastOutput) && fabs(pid->lastOutput - pid->output) > pid->slewRate)
    pid->output -= pid->slewRate;
  
  else if((pid->output < pid->lastOutput) && fabs(pid->lastOutput - pid->output) > pid->slewRate)
    pid->output += pid->slewRate;
   
  pid->lastError = pid->error;
  pid->lastTime = Brain.timer(timeUnits::msec);

  pid->lastOutput = pid->output;
  return pid->output;
}

void printPIDValues(pidStruct_t *pid)
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Error: %f\n", pid->error);
  Brain.Screen.newLine();
  Brain.Screen.print("Integral: %f\n", pid->integral);
  Brain.Screen.newLine();
  Brain.Screen.print("derivative: %f\n", pid->derivative);
  Brain.Screen.newLine();
  Brain.Screen.print("output: %f", pid->output);

  wait(20, msec);
}