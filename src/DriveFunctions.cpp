/********************************************************************
 *    DriveFunctions.cpp
 * This is where all the drive code lives for user and auton
 * Make sure you have done all the configs in DriveFunctionsConfig.h
 *******************************************************************/

#include "DriveFunctionsConfig.h"
using namespace vex;

#ifdef CHASSIS_4_MOTOR_INLINE
  motor FrontLeft = motor(FrontLeftPort, GEAR_SET, true);
  motor BackLeft = motor(BackLeftPort, GEAR_SET, true);
  motor FrontRight = motor(FrontRightPort, GEAR_SET, false);
  motor BackRight = motor(BackRightPort, GEAR_SET, false);

#elif defined(CHASSIS_2_MOTOR_INLINE)
  motor DriveLeft = motor(DriveLeftPort, GEAR_SET, false);
  motor DriveRight = motor(DriveRightPort, GEAR_SET, true);

#elif defined(CHASSIS_X_DRIVE)
  //coming soon!
#endif

#ifdef GYRO
  inertial myGyro = inertial(GYRO_PORT);
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
  wait(1, sec);

#if defined(PID)
  float motorPower = 0;

  #if defined (CHASSIS_2_MOTOR_INLINE)
    float DriveR_Power = 0;
    float DriveL_Power = 0;

    pidStruct_t driveL_PID;
    pidStruct_t driveR_PID;

    pidInit(&driveL_PID, 40, 1, 0, 10, 10);
    pidInit(&driveR_PID, 40, 1, 0, 10, 10);

    DriveRight.resetRotation();
    DriveLeft.resetRotation();

  #elif defined(CHASSIS_4_MOTOR_INLINE)
    FrontLeft.resetRotation();
    FrontRight.resetRotation();
    BackLeft.resetRotation();
    BackRight.resetRotation();
  #endif
 
  printPIDValues(&driveR_PID);

  do
  {
    while(fabs(driveR_PID.error) > 0.05 || fabs(driveL_PID.error) > 0.05)
    {
    #if defined (CHASSIS_2_MOTOR_INLINE)
      printPIDValues(&driveR_PID);
      DriveR_Power = (velocity/100.0f) * pidCalculate(&driveR_PID, rotations, DriveRight.rotation(rev));
      DriveL_Power = (velocity/100.0f) * pidCalculate(&driveL_PID, rotations, DriveLeft.rotation(rev));

      DriveRight.spin(forward, DriveR_Power, pct);
      DriveLeft.spin(forward, DriveL_Power, pct);

    #elif defined (CHASSIS_4_MOTOR_INLINE)
      FrontRight.spin(forward, motorPower, pct);
      FrontLeft.spin(forward, motorPower, pct);
      BackLeft.spin(forward, motorPower, pct);
      BackRight.spin(forward, motorPower, pct);
    #endif
    }

    for(int i = 100; i > 0; i--)
    {
    #ifdef CHASSIS_4_MOTOR_INLINE
      FrontLeft.stop();
      BackLeft.stop();
      FrontRight.stop();
      BackRight.stop();

    #elif defined(CHASSIS_2_MOTOR_INLINE)
      DriveRight.stop(brakeType::coast);
      DriveLeft.stop(brakeType::coast);
    #endif
      wait(1, msec);
    }
       // task::sleep(300);
    pidCalculate(&driveR_PID, rotations, DriveRight.rotation(rev));
    pidCalculate(&driveL_PID, rotations, DriveLeft.rotation(rev));

  }while(fabs(driveR_PID.error) > 0.05 || fabs(driveL_PID.error) > 0.05);

#elif !defined (PID)
  #if defined (CHASSIS_2_MOTOR_INLINE)
    DriveRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    DriveLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  #elif defined (CHASSIS_4_MOTOR_INLINE)
    FrontLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    BackLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    FrontRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    BackRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
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
  DriveRight.stop(brakeType::hold);
  DriveLeft.stop(brakeType::hold);
#endif
}


void moveRotate(int16_t degrees, int velocity)
{
  float arcLength = (degrees/360.0f) * CIRCUMFERENCE;
  float rotFactor = ROTATION_FACTOR;
  float rotations = arcLength / rotFactor;

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("rotations: %f", rotations);
  Brain.Screen.newLine();
  Brain.Screen.print("arc length: %f", arcLength);
  Brain.Screen.newLine();
  Brain.Screen.print("Hyp: %f", HYPOTENUSE);
  Brain.Screen.newLine();
  Brain.Screen.print("Circ: %f", CIRCUMFERENCE);
  Brain.Screen.newLine();
  Brain.Screen.print("rotations factor: %f", ROTATION_FACTOR);
  
  wait(1, sec);

#if defined(PID) 
  #ifdef GYRO
    myGyro.startCalibration();
    while(myGyro.isCalibrating());
    myGyro.resetHeading();
  #endif

  #ifdef CHASSIS_2_MOTOR_INLINE
    DriveLeft.resetRotation();
    DriveRight.resetRotation();
  #endif

  pidStruct_t rotatePID;
  pidInit(&rotatePID, 0.8, 0.01, 0, 1, 10);
  rotatePID.error = 10;
  float motorPower = 0;

  printPIDValues(&rotatePID);

  while(fabs(rotatePID.error) > 0.1)
  {
    printPIDValues(&rotatePID);

  #if defined(PID) && defined (GYRO)
    motorPower = (velocity/100.0f) * pidCalculate(&rotatePID, degrees, myGyro.heading(rotationUnits::deg));
  #elif defined(PID) && !(GYRO)
    #ifdef CHASSIS_4_MOTOR_INLINE
      motorPower = (velocity/100.0f) * pidCalculate(&rotatePID, rotations, FrontRight.rotation(rev));
    #elif defined CHASSIS_2_MOTOR_INLINE
      motorPower = (velocity/100.0f) * pidCalculate(&rotatePID, rotations, DriveLeft.rotation(rev));
    #endif
  #endif

  #ifdef CHASSIS_4_MOTOR_INLINE
    FrontRight.spin(forward, motorPower, pct);
    FrontLeft.spin(reverse, motorPower, pct);
    BackRight.spin(forward, motorPower, pct);
    BackLeft.spin(reverse, motorPower, pct);

  #elif defined CHASSIS_2_MOTOR_INLINE
    DriveRight.spin(reverse, motorPower, pct);
    DriveLeft.spin(forward, motorPower, pct);
  #endif

    wait(10, msec);
  }

#elif !defined(PID) && !defined(GYRO)
  #ifdef CHASSIS_4_MOTOR_INLINE
    FrontLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    BackLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    FrontRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    BackRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);

  #elif defined CHASSIS_2_MOTOR_INLINE
    DriveRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    DriveLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  #endif
#endif
}