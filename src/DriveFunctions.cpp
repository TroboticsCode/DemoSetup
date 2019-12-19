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
  motor DriveLeft = motor(DriveLeftPort, GEAR_SET, true);
  motor DriveRight = motor(DriveRightPort, GEAR_SET, false);

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
  float rotations = distance/ROTATION_FACTOR;

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