#ifndef AUTONS_H
#define AUTONS_H

//always include vex.h because it has all the robot setup declarations
//as well as the API for the code.
#include "vex.h"

#define NONE 0
#define AutonR 1
#define AutonB 2

extern int state;

//list the names of your autonomous programs here. the () assumes that there
//are no parameters for these
void Auton1();
void Auton2();
void cycle_autons();

//Do not write the programs here. This is just a list of them. Write them 
//down in the src folder as .cpp files

#endif
