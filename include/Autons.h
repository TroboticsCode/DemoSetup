#ifndef AUTONS_H
#define AUTONS_H

//always include vex.h because it has all the robot setup declarations
//as well as the API for the code.
#include "vex.h"

//no touchy - this is the data structure used to 
//store function names and pointers
typedef struct
{
  char programName[16];
  void  (*fp)(void);
}autonsTuple;

extern uint8_t autonIndex;
extern autonsTuple autons[];
extern const uint8_t numAutons;

//auton support function prototypes
void cycle_autons(void);
void Paint_Screen(void);
void auton_runner(void);

#endif
