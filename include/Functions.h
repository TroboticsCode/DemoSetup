#ifndef FUNCTIONS_H
#define FUNCTIONS_H

//always include vex.h because it has all the robot setup declarations
//as well as the API for the code.
#include "vex.h"

void oneMotor(int speed, int spins);
void twoMotors(int speed, int spins);
void Paint_Screen();

#endif