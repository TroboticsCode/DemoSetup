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

//slewing
  if((pid->output > pid->lastOutput) && fabs(pid->lastOutput - pid->output) > pid->slewRate)
    pid->output = pid->lastOutput + pid->slewRate;
  
  else if((pid->output < pid->lastOutput) && fabs(pid->lastOutput - pid->output) > pid->slewRate)
    pid->output = pid->lastOutput - pid->slewRate;
  
  //constrain output
  if(pid->output > 100)
    pid->output = 100; 
  if(pid->output < -100)
    pid->output = -100;

  pid->lastError = pid->error;
  pid->lastTime = Brain.timer(timeUnits::msec);

  //moving averages filter
  if(pid->numIterations > SAMPLES_AVG) //array has been filled, calculate average
  {
    //make room for the next sample
    for(int i = 0; i < (SAMPLES_AVG-1); i++)
    {
      pid->errorSamples[i] = pid->errorSamples[i+1];
    }

    //place the new sample at the end of the array
    pid->errorSamples[SAMPLES_AVG-1] = pid->error;

    //add all samples and et average
    for(int i = 0; i < SAMPLES_AVG; i++)
    {
      pid->avgError += pid->errorSamples[i];
    }
    pid->avgError /= (float)SAMPLES_AVG;
  }
  else //fill the array
  {
    pid->errorSamples[pid->numIterations] = pid->error;
    pid->numIterations++;
    pid->avgError = pid->error;
  }

  pid->lastOutput = pid->output;
  return pid->output;
}

void printPIDValues(pidStruct_t *pid)
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Error: %f\n", pid->error);
  Brain.Screen.newLine();
  Brain.Screen.print("Average Error: %f\n", pid->avgError);
  Brain.Screen.newLine();
  Brain.Screen.print("Integral: %f\n", pid->integral);
  Brain.Screen.newLine();
  Brain.Screen.print("derivative: %f\n", pid->derivative);
  Brain.Screen.newLine();
  Brain.Screen.print("output: %f", pid->output);

  wait(20, msec);
}