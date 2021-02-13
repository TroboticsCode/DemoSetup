// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertial12           inertial      12              
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       JBormann                                                  */
/*    Created:      Nov. 2019                                                 */
/*    Description:  Trobotics Template File                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#define CENTER_X 316/2.0f
#define CENTER_Y 212/2.0f

#include "vex.h"
#include "Autons.h"
#include "Functions.h"
#include "DriveFunctionsConfig.h"
#include "VisionSensor.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) 
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  Controller1.ButtonR1.pressed(cycle_autons);
  Brain.Screen.pressed(cycle_autons);
  return;
}

void autonomous(void) 
{
 switch (state)
  {
    case NONE:
    break;

    case AutonR:    
      Auton1();
    break;

    case AutonB:
  
    break;
          
    // Default = NO autonomous
    default:
    break;
  }
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) { 
  //add local user control variables here:
  int32_t numObjects;
  Brain.Screen.clearScreen();
  Brain.Screen.print("user code running");
  Brain.Screen.newLine();
  
  //User control code here, inside the loop:
  //remove existing demo code and replace with you own! Then remove this comment
  while (1) {
    numObjects = Vision1.takeSnapshot(RED_BALL);
        
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("centerX: %d\n", Vision1.largestObject.centerX);
    Brain.Screen.newLine();
    Brain.Screen.print("centerY: %d", Vision1.largestObject.centerY);
    Brain.Screen.newLine();
    Brain.Screen.clearLine();

    if(Vision1.largestObject.exists)
    {
      if(Vision1.largestObject.centerX > (CENTER_X + 20))
      {
        Brain.Screen.print("turn right");
        BackRight.spin(directionType::rev, 20, velocityUnits::pct);
        BackLeft.spin(directionType::fwd, 20, velocityUnits::pct);
        FrontRight.spin(directionType::rev, 20, velocityUnits::pct);
        FrontLeft.spin(directionType::fwd, 20, velocityUnits::pct);
      }
      else if(Vision1.largestObject.centerX < (CENTER_X - 20))
      {
        Brain.Screen.print("turn left ");
        BackRight.spin(directionType::fwd, 20, velocityUnits::pct);
        BackLeft.spin(directionType::rev, 20, velocityUnits::pct);
        FrontRight.spin(directionType::fwd, 20, velocityUnits::pct);
        FrontLeft.spin(directionType::rev, 20, velocityUnits::pct);        
      }
      else
      {
        Brain.Screen.print("Dont move");
        BackRight.stop();
        BackLeft.stop();
        FrontRight.stop();
        FrontLeft.stop();
      }
    }
    else
    {
      Brain.Screen.print("nothing detected");
      BackRight.stop();
      BackLeft.stop();
      FrontRight.stop();
      FrontLeft.stop();
    }

    wait(20, msec); // Sleep the task for a short amount of time to
  }
}


// Main will set up the competition functions and callbacks.

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
