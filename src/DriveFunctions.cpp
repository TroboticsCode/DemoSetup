/********************************************************************
 *    DriveFunctions.cpp
 * This is where all the drive code lives for user and auton
 * Make sure you have done all the configs in DriveFunctionsConfig.h
 *******************************************************************/

#include "DriveFunctionsConfig.h"
using namespace vex;

#ifdef CHASSIS_4_MOTOR_INLINE
  motor FrontLeft = motor(FrontLeftPort, GEAR_SET, false);
  motor BackLeft = motor(BackLeftPort, GEAR_SET, false);
  motor FrontRight = motor(FrontRightPort, GEAR_SET, true);
  motor BackRight = motor(BackRightPort, GEAR_SET, true);

#elif defined(CHASSIS_2_MOTOR_INLINE)
  motor DriveLeft = motor(DriveLeftPort, GEAR_SET, false);
  motor DriveRight = motor(DriveRightPort, GEAR_SET, true);

#elif defined(CHASSIS_X_DRIVE)
  //coming soon!
#endif

#ifdef GYRO
  inertial myGyro = inertial(GYRO_PORT);
#endif

////////////////User Drive Functions/////////////////////////
void userDrive(void)
{
#ifdef ARCADE_CONTROL
  int32_t horizontalAxis = Controller1.HORIZONTALAXIS.value()/2;
  int32_t verticalAxis = Controller1.VERTICALAXIS.value()/2;
  
  #ifdef CHASSIS_2_MOTOR_INLINE
    DriveRight.setBrake(brakeType::coast);
    DriveLeft.setBrake(brakeType::coast);

    DriveRight.spin(directionType::fwd, (verticalAxis - horizontalAxis), velocityUnits::pct);
    DriveLeft.spin(directionType::fwd, (verticalAxis + horizontalAxis), velocityUnits::pct);
  
  #elif defined CHASSIS_4_MOTOR_INLINE
    BackRight.setBrake(brakeType::coast);
    FrontRight.setBrake(brakeType::coast);
    BackLeft.setBrake(brakeType::coast);
    FrontLeft.setBrake(brakeType::coast);

    BackRight.spin(directionType::fwd, (verticalAxis - (horizontalAxis)), velocityUnits::pct);
    BackLeft.spin(directionType::fwd, (verticalAxis + (horizontalAxis)), velocityUnits::pct);
    FrontRight.spin(directionType::fwd, (verticalAxis - (horizontalAxis)), velocityUnits::pct);
    FrontLeft.spin(directionType::fwd, (verticalAxis + (horizontalAxis)), velocityUnits::pct);
  #endif

#elif defined TANK_CONTROL
  int32_t leftAxis = Controller1.LEFTAXIS.value();
  int32_t rightAxis = Controller1.RIGHTAXIS.value();
  #ifdef CHASSIS_2_MOTOR_INLINE
    DriveLeft.spin(directionType::fwd, leftAxis, velocityUnits::pct);
    DriveRight.spin(directionType::fwd, rightAxis, velocityUnits::pct);
  
  #elif defined CHASSIS_4_MOTOR_INLINE
    BackLeft.spin(directionType::fwd, leftAxis, velocityUnits::pct);
    BackRight.spin(directionType::fwd, rightAxis, velocityUnits::pct);
    FrontLeft.spin(directionType::fwd, leftAxis, velocityUnits::pct);
    FrontRight.spin(directionType::fwd, rightAxis, velocityUnits::pct);
  #endif
#endif
}

////////////////Auton Drive Functions////////////////////////
/**************************************************
 * @brief: moves the robot forward or back
 *    at a given speed
 * @TODO: Fix comments about distance and velocity
 * @param distance: how far to move in inches, absolute value
 * @param velocity: how fast to move, signed value
 *                  sign determines direction
 **************************************************/
static double lin_kP = 0;
static double lin_kI = 0;
static double lin_kD = 0;
static double lin_slewRate = 20;
static int    lin_minDT = 10;

void moveLinear(float distance, int velocity, uint32_t timeOut)
{
  float rotations = distance * (1/((float)ROTATION_FACTOR));
  Brain.Screen.print("Rotations to turn: %f", rotations);
  Brain.Screen.newLine();
  Brain.Screen.print("Rotation Factor: %f", ROTATION_FACTOR);


#if defined(PID)
  float DriveR_Power = 0;
  float DriveL_Power = 0;

  pidStruct_t driveL_PID;
  pidStruct_t driveR_PID;

  pidInit(&driveL_PID, lin_kP, lin_kI, lin_kD, lin_slewRate, lin_minDT);
  pidInit(&driveR_PID, lin_kP, lin_kI, lin_kD, lin_slewRate, lin_minDT);

  #if defined (CHASSIS_2_MOTOR_INLINE)
    DriveRight.resetRotation();
    DriveLeft.resetRotation();
  #elif defined(CHASSIS_4_MOTOR_INLINE)
    FrontLeft.resetRotation();
    FrontRight.resetRotation();
    BackLeft.resetRotation();
    BackRight.resetRotation();

    double leftRevAvg  = 0;
    double rightRevAvg = 0;

    uint64_t startTime = Brain.timer(timeUnits::msec);
  #endif
 
  printPIDValues(&driveR_PID);

  do
  {
    #if defined (CHASSIS_2_MOTOR_INLINE)
      printPIDValues(&driveR_PID);
      DriveR_Power = (velocity/100.0f) * pidCalculate(&driveR_PID, rotations, DriveRight.rotation(rev) / 100.0);
      DriveL_Power = (velocity/100.0f) * pidCalculate(&driveL_PID, rotations, DriveLeft.rotation(rev) / 100.0);

      DriveRight.spin(forward, 12 * DriveR_Power, voltageUnits::volt);
      DriveLeft.spin(forward, 12 * DriveL_Power, voltageUnits::volt);

    #elif defined (CHASSIS_4_MOTOR_INLINE)
      printPIDValues(&driveR_PID);

      leftRevAvg = (BackLeft.rotation(rev) + FrontLeft.rotation(rev)) / 2.0;
      rightRevAvg = (BackRight.rotation(rev) + FrontRight.rotation(rev)) / 2.0;

      DriveR_Power = (velocity/100.0f) * (pidCalculate(&driveR_PID, rotations, rightRevAvg) / 100.0);
      DriveL_Power = (velocity/100.0f) * (pidCalculate(&driveL_PID, rotations, leftRevAvg) / 100.0);

      FrontRight.spin(forward, 12 * DriveR_Power, voltageUnits::volt);
      FrontLeft.spin(forward, 12 * DriveL_Power, voltageUnits::volt);
      BackLeft.spin(forward, 12 * DriveL_Power, voltageUnits::volt);
      BackRight.spin(forward, 12 * DriveR_Power, voltageUnits::volt);
    #endif
    
  }while((fabs(driveR_PID.avgError) > 0.03 || fabs(driveL_PID.avgError) > 0.03) && (Brain.timer(timeUnits::msec) - startTime < timeOut));

#elif !defined (PID)
  #if defined (CHASSIS_2_MOTOR_INLINE)
    DriveRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    DriveLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
  #elif defined (CHASSIS_4_MOTOR_INLINE)
    FrontLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    BackLeft.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    FrontRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    BackRight.rotateFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, true);
  #endif
#endif
}


void moveStop(void)
{
#ifdef CHASSIS_4_MOTOR_INLINE
  FrontLeft.stop(brakeType::hold);
  BackLeft.stop(brakeType::hold);
  FrontRight.stop(brakeType::hold);
  BackRight.stop(brakeType::hold);

#elif defined(CHASSIS_2_MOTOR_INLINE)
  DriveRight.stop(brakeType::hold);
  DriveLeft.stop(brakeType::hold);
#endif
}

static double rot_kP = 0;
static double rot_kI = 0;
static double rot_kD = 0;
static double rot_slewRate = 20;
static int    rot_minDT = 10;

void moveRotate(int16_t degrees, int velocity, uint32_t timeOut)
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
  

#if defined(PID) 
  #ifdef GYRO
    //myGyro.calibrate();
    //while(myGyro.isCalibrating());
    myGyro.resetRotation();
  #endif

  #ifdef CHASSIS_2_MOTOR_INLINE
    DriveLeft.resetRotation();
    DriveRight.resetRotation();
  #elif defined CHASSIS_4_MOTOR_INLINE
    FrontLeft.resetRotation();
    FrontRight.resetRotation();
    BackLeft.resetRotation();
    BackRight.resetRotation();

    double leftRevAvg  = 0;
    double rightRevAvg = 0;

    uint64_t startTime = Brain.timer(timeUnits::msec);
  #endif

  #if !defined GYRO
    pidStruct_t rotateR_PID;
    pidStruct_t rotateL_PID;

    pidInit(&rotateR_PID, rot_kP, rot_kI, rot_kD, rot_slewRate, rot_minDT);
    pidInit(&rotateL_PID, rot_kP, rot_kI, rot_kD, rot_slewRate, rot_minDT);

    float DriveR_Power = 0;
    float DriveL_Power = 0;

  #elif defined GYRO
    pidStruct_t rotatePID;
    pidInit(&rotatePID, rot_kP, rot_kI, rot_kD, rot_slewRate, rot_minDT);

    float motorPower = 0;
  #endif

  do
  {
  #if defined (GYRO)
    motorPower = (velocity/100.0f) * pidCalculate(&rotatePID, degrees, myGyro.rotation(rotationUnits::deg));
  #elif !defined (GYRO)
    #ifdef CHASSIS_4_MOTOR_INLINE
      leftRevAvg = (BackLeft.rotation(rev) + FrontLeft.rotation(rev)) / 2.0;
      rightRevAvg = (BackRight.rotation(rev) + FrontRight.rotation(rev)) / 2.0;

      DriveL_Power = (velocity/100.0f) * pidCalculate(&rotateL_PID, rotations, -1.0 * leftRevAvg) / 100.0f;
      DriveR_Power = (velocity/100.0f) * pidCalculate(&rotateR_PID, rotations, rightRevAvg) / 100.0f;
    #elif defined CHASSIS_2_MOTOR_INLINE
      DriveL_Power = (velocity/100.0f) * pidCalculate(&rotateL_PID, rotations, DriveLeft.rotation(rev) / 100.0f);
      DriveR_Power = (velocity/100.0f) * pidCalculate(&rotateR_PID, rotations, -1.0f * DriveRight.rotation(rev) / 100.0f);
    #endif
  #endif

  #if defined (GYRO)
    printPIDValues(&rotatePID);
    #ifdef CHASSIS_4_MOTOR_INLINE
      FrontRight.spin(reverse, 12 * motorPower, voltageUnits::volt);
      FrontLeft.spin(forward, 12 * motorPower, voltageUnits::volt);
      BackLeft.spin(forward, 12 * motorPower, voltageUnits::volt);
      BackRight.spin(reverse, 12 * motorPower, voltageUnits::volt);

    #elif defined CHASSIS_2_MOTOR_INLINE
      DriveRight.spin(reverse, 12 * motorPower, voltageUnits::volt);
      DriveLeft.spin(forward, 12 * motorPower, voltageUnits::volt);
    #endif

  #else 
    printPIDValues(&rotateR_PID);
    #ifdef CHASSIS_4_MOTOR_INLINE
      FrontRight.spin(forward, 12 * DriveR_Power, voltageUnits::volt);
      FrontLeft.spin(reverse, 12 * DriveL_Power, voltageUnits::volt);
      BackLeft.spin(reverse, 12 * DriveL_Power, voltageUnits::volt);
      BackRight.spin(forward, 12 * DriveR_Power, voltageUnits::volt);

    #elif defined CHASSIS_2_MOTOR_INLINE
      DriveRight.spin(reverse, 12 * DriveR_Power, voltageUnits::volt);
      DriveLeft.spin(forward, 12 * DriveL_Power, voltageUnits::volt);
    #endif
  #endif

  wait(10, msec);

  #if defined GYRO
  }while((fabs(rotatePID.avgError) > 0.3) && (Brain.timer(timeUnits::msec) - startTime < timeOut)); //error in degrees
  #elif !defined GYRO
  }while((fabs(rotateR_PID.avgError) > 0.1 || fabs(rotateL_PID.avgError) > 0.1) && (Brain.timer(timeUnits::msec) - startTime < timeOut)); //error in units of revs
  #endif
  //end do-while

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

#if defined(PID)
  void setLinGains(double kP, double kI, double kD, double slewRate, int minDT)
  {
    lin_kP = kP;
    lin_kI = kI;
    lin_kD = kD;
    lin_slewRate = slewRate;
    lin_minDT = minDT;
  }

  void setRotGains(double kP, double kI, double kD, double slewRate, int minDT)
  {
    rot_kP = kP;
    rot_kI = kI;
    rot_kD = kD;
    rot_slewRate = slewRate;
    rot_minDT = minDT;
  }
#endif
