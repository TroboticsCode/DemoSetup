#include "Autons.h"

//this is where we need to put the case names of all the autonomous programs we use 
//the number are going to be referenced by the case switching function

//this is a placeholder for another auton. we can add as many as we like
//#define ******** 3


//intitial state = NONE;
int state = 0;
char state_name[] = "NONE";

void Paint_Screen(){
  Controller1.Screen.setCursor(3,1);
  Controller1.Screen.clearLine(3);
  Controller1.Screen.print("%s", state_name);
}

void cycle_autons(void)
{
    if (state == 0)
    {
        state = AutonR;
        strcpy(state_name, "Auton_1");
    }
    else if (state == AutonR)
    {
         state =   AutonB;
         strcpy(state_name, "Auton_2");
    }
    
    else if (state == AutonB)
    {
        state = NONE;
        strcpy(state_name, "NONE");
    }
    Paint_Screen();
}