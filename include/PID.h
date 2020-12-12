/********************************************************* 
*    PID.h
* This file is to be used with PID.cpp
* This is a C implementation of closed loop control
* An instance of it may be created to control any motor
* or drive command
********************************************************/
#ifndef PID_H
#define PID_H

#include "vex.h"
#include <math.h>
#include <stdlib.h>
using namespace vex;

#define SAMPLES_AVG 10

typedef struct
{
  double kP = 0;
  double kD = 0;
  double kI = 0;
  double slewRate = 0;

  int minDt = 10;
  int numIterations = 0;

  double error = 10;
  double derivative = 0;
  double integral = 0;
  double lastError = 0;
  double errorSamples[SAMPLES_AVG] = {0};
  double avgError = 10;
  double lastTime = 0;
  double output = 0;
  double lastOutput = 0;
} pidStruct_t;

void pidInit(pidStruct_t* pid, double kP, double kI, double kD, double slewRate, int minDt);
double pidCalculate(pidStruct_t* pid, double target, double current);
void printPIDValues(pidStruct_t *pid);

#endif