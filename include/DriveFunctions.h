#ifndef DRIVE_FUNCTIONS_H
#define DRIVE_FUNCTIONS_H

//always include vex.h because it has all the robot setup declarations
//as well as the API for the code.
#include "vex.h"

//enter the prototypes for drive functions here
void oneMotor(int speed, int spins);
void twoMotors(int speed, int spins);


#endif