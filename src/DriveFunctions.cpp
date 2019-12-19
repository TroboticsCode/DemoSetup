/********************************************************************
 *    DriveFunctions.cpp
 * This is where all the drive code lives for user and auton
 * Make sure you have done all the configs in DriveFunctionsConfig.h
 *******************************************************************/

#include "Functions.h"
#include "DriveFunctionsConfig.h"
using namespace vex;

#ifdef CHASSIS_4_MOTOR_INLINE
  motor FrontLeft = motor(FrontLeftPort, GEAR_SET, true);
  motor RearLeft = motor(RearLeftPort, GEAR_SET, true);
  motor FrontRight = motor(FrontRightPort, GEAR_SET, false);
  motor RearRight = motor(RearRightPort, GEAR_SET, false);

#elif defined(CHASSIS_2_MOTOR_INLINE)
  motor DriveLeft = motor(DriveLeftPort, GEAR_SET, false);
  motor DriveRight = motor(DriveRightPort, GEAR_SET, true);

#elif defined(CHASSIS_X_DRIVE)
  //coming soon!
#endif

/**************************************************
 * @brief: moves the robot forward or back
 *    at a given speed
 *
 * @param distance: how far to move in inches, absolute value
 * @param velocity: how fast to move, signed value
 *                  sign determines direction
 **************************************************/
void moveLinear(float distance, int velocity)
{
  float rotations = distance * (1/((float)ROTATION_FACTOR));
  Brain.Screen.print("Rotations to turn: %f", rotations);
  Brain.Screen.newLine();
  Brain.Screen.print("Rotation Factor: %f", ROTATION_FACTOR);

#ifdef CHASSIS_4_MOTOR_INLINE
  FrontLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  BackLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  FrontRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  BackRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);

#elif defined(CHASSIS_2_MOTOR_INLINE)
  #ifdef PID
  pidStruct_t drivePID;
  pidInit(&drivePID, 0.6, 0.5, 10);
  drivePID.error = 10;

  float motorPower = 0;
  DriveRight.resetRotation();
  DriveLeft.resetRotation();
 
  printPIDValues(&drivePID);

  while(drivePID.error > 0.05)
  {
    //Brain.Screen.print("\nIn the loop");
    //Brain.Screen.newLine();
    printPIDValues(&drivePID);
    motorPower = 100 * pidCalculate(&drivePID, rotations, DriveRight.rotation(rev));
    DriveRight.spin(forward, motorPower, pct);
    DriveLeft.spin(forward, motorPower, pct);

    wait(10, msec);
  }

  #elif !defined PID
  DriveRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  DriveLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  #endif
#endif
}
void moveStop(void)
{
#ifdef CHASSIS_4_MOTOR_INLINE
  FrontLeft.stop();
  BackLeft.stop();
  FrontRight.stop();
  BackRight.stop();

#elif defined(CHASSIS_2_MOTOR_INLINE)
  DriveRight.stop();
  DriveLeft.stop();
#endif
}


void moveRotate(uint16_t degrees, int velocity)
{
  float arcLength = (degrees/360) * CIRCUMFERENCE;
  float rotations = arcLength/ROTATION_FACTOR;
#ifdef CHASSIS_4_MOTOR_INLINE
  FrontLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  BackLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  FrontRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  BackRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);

#elif defined(CHASSIS_2_MOTOR_INLINE)
  DriveRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  DriveLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);

#endif
}