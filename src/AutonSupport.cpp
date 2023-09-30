/***********************************
 * AutonSupport.cpp
 * This file contains support
 * functions for selecting auton
 * routines
 * Update as needed, do not delete
***********************************/

#include "Autons.h"

uint8_t autonIndex = 0;

/* void Paint_Screen()
* brief: this function will update the brain's display to show the currently selected auton program
*/
void Paint_Screen(void)
{
  Controller1.Screen.setCursor(3,1);
  Controller1.Screen.clearLine(3);
  Controller1.Screen.print("%s", autons[autonIndex].programName);
}

/*void cycle_autons
* brief: this function is called when a button on the controller is pressed to advance to the 
*          next auton routine
*/
void cycle_autons(void)
{
  autonIndex++;
  autonIndex %= numAutons; //wrap back to zero
  
  Paint_Screen();
}

/*void auton_runner
 * breief: this function runs whatever function is 
 *         pointed to by the function pointer in 
 *         the autons array
 */
void auton_runner(void)
{
  void (*auton)(void) = autons[autonIndex].fp;
  auton();
}
