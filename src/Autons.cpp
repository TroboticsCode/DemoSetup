#include "vex.h"
#include "Autons.h"
#include "Functions.h"
#include "DriveFunctionsConfig.h"

//add function prototypes for auton routines here
void noAuton();
void Auton1();
void Auton2();

//add function names and function pointers here
//follow the existing sytax, replacing the string
//with function name to show on the controller screen
autonsTuple autons[] = {{"NONE",    noAuton},
                        {"AUTON_1", Auton1},
                        {"AUTON_2", Auton2}};

const uint8_t numAutons = sizeof(autons)/sizeof(autonsTuple);


//Put your auton routines in here
void noAuton()
{
  //dummy function for running no auton
  //LEAVE IN PLACE!
}

void Auton1()
{
  //setRotGains(0, 0, 0, 20, 10); //update PID gains to tune robot
  //setLinGains(0, 0, 0, 20, 10);

  moveLinear(24, 100, 10000);
}

void Auton2()
{

}