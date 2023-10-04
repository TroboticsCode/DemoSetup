/********************************************************************
 *    DriveFunctions.cpp
 * This is where all the drive code lives for user and auton
 * Make sure you have done all the configs in DriveFunctionsConfig.h
 *******************************************************************/

#include "DriveFunctionsConfig.h"
using namespace vex;

void updateDriveMotors(int leftDriveVal, int rightDriveVal);
void updateDriveMotorVolts(int leftVoltage, int rightVoltage);
void setDriveBrake(brakeType brake_type);
float getMotorAvgRotations(vector<motor> motorGroup);
void resetDriveRotations(void);

vector<motor> leftDriveMotors;
vector<motor> rightDriveMotors;

void initDriveMotors()
{
  for(uint8_t i = 0; i < numDriveMotors; i++)
  {
    leftDriveMotors.push_back(motor(leftDrivePorts[i], GEAR_SET, false));
    rightDriveMotors.push_back(motor(rightDrivePorts[i], GEAR_SET, true));
  }

  setDriveBrake(brakeType::coast);

#ifdef GYRO
  inertial myGyro = inertial(GYRO_PORT);
#endif
}

////////////////User Drive Functions/////////////////////////
void userDrive(void)
{
  setDriveBrake(coast);

#ifdef CHASSIS_INLINE_DRIVE
  #ifdef ARCADE_CONTROL
    int32_t horizontalAxis = Controller1.HORIZONTALAXIS.value();
    int32_t verticalAxis = Controller1.VERTICALAXIS.value();
    
    updateDriveMotors((verticalAxis + horizontalAxis), (verticalAxis - horizontalAxis));

  #elif defined TANK_CONTROL
    int32_t leftAxis = Controller1.LEFTAXIS.value();
    int32_t rightAxis = Controller1.RIGHTAXIS.value();

    updateDriveMotors(leftAxis, rightAxis);
  #endif
#elif defined CHASSIS_X_DRIVE
  int32_t horizontalAxis = Controller1.HORIZONTALAXIS.value()/2;
  int32_t verticalAxis = Controller1.VERTICALAXIS.value()/2;
  int32_t rotateAxis = Controller1.ROTATIONAXIS.value()/2;

  if(abs(horizontalAxis) < DEADZONE)
    horizontalAxis = 0;
  if(abs(verticalAxis) < DEADZONE)
    verticalAxis = 0;
  if(abs(rotateAxis) < DEADZONE)
    rotateAxis = 0;

    rightDriveMotors[1].spin(directionType::fwd, (verticalAxis + horizontalAxis - rotateAxis), velocityUnits::pct);
    leftDriveMotors[1].spin(directionType::fwd, (verticalAxis - horizontalAxis + rotateAxis), velocityUnits::pct);
    rightDriveMotors[0].spin(directionType::fwd, (verticalAxis - horizontalAxis - rotateAxis), velocityUnits::pct);
    leftDriveMotors[0].spin(directionType::fwd, (verticalAxis + horizontalAxis + rotateAxis), velocityUnits::pct);

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

  /*
   * X drive angles wheels at 45deg so 
   *  the robot will move further per rotation
   *  by a factor of square root of 2 => 1.414
   */
  #ifdef CHASSIS_X_DRIVE
    rotations /= sqrt(2);
  #endif

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

  resetDriveRotations();

  double leftRevAvg  = 0;
  double rightRevAvg = 0;

  uint64_t startTime = Brain.timer(timeUnits::msec);
 
  printPIDValues(&driveR_PID);

  do
  {
    printPIDValues(&driveR_PID);

    leftRevAvg = getMotorAvgRotations(leftDriveMotors);
    rightRevAvg = getMotorAvgRotations(rightDriveMotors);

    DriveR_Power = (velocity/100.0f) * (pidCalculate(&driveR_PID, rotations, rightRevAvg) / 100.0);
    DriveL_Power = (velocity/100.0f) * (pidCalculate(&driveL_PID, rotations, leftRevAvg) / 100.0);
    
    updateDriveMotorVolts(12*DriveL_Power, 12*DriveR_Power);
    
  }while((fabs(driveR_PID.avgError) > 0.03 || fabs(driveL_PID.avgError) > 0.03) && (Brain.timer(timeUnits::msec) - startTime < timeOut));

#elif !defined (PID)
  for(uint8_t i = 0; i<numDriveMotors; i++)
  {
    leftDriveMotors[i].spinFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);

    if(i == numDriveMotors-1) //this is the last motor in the array and needs to be blocking so we complete the move
    {
      rightDriveMotors[i].spinFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, true);
    }
    else
    {
      rightDriveMotors[i].spinFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    }

  }
#endif
}


void moveStop(brakeType brake_type)
{
  setDriveBrake(brake_type);
  updateDriveMotors(0, 0);
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

  resetDriveRotations();  

  double leftRevAvg  = 0;
  double rightRevAvg = 0;

  uint64_t startTime = Brain.timer(timeUnits::msec);

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
    leftRevAvg = getMotorAvgRotations(leftDriveMotors);
    rightRevAvg = getMotorAvgRotations(rightDriveMotors);

    DriveL_Power = (velocity/100.0f) * pidCalculate(&rotateL_PID, rotations, -1.0 * leftRevAvg) / 100.0f;
    DriveR_Power = (velocity/100.0f) * pidCalculate(&rotateR_PID, rotations, rightRevAvg) / 100.0f;    
  #endif

  #if defined (GYRO)
    printPIDValues(&rotatePID);
    updateDriveMotorVolts((12*motorPower), (-12*motorPower));
  #else 
    printPIDValues(&rotateR_PID);
    updateDriveMotorVolts(12*DriveL_Power, 12*DriveR_Power);
  #endif

  wait(10, msec);

  #if defined GYRO
  }while((fabs(rotatePID.avgError) > 0.3) && (Brain.timer(timeUnits::msec) - startTime < timeOut)); //error in degrees
  #elif !defined GYRO
  }while((fabs(rotateR_PID.avgError) > 0.1 || fabs(rotateL_PID.avgError) > 0.1) && (Brain.timer(timeUnits::msec) - startTime < timeOut)); //error in units of revs
  #endif
  //end do-while

#elif !defined(PID) && !defined(GYRO)
  Brain.Screen.newLine();
  Brain.Screen.print("Doing basing rotate");
  for(uint8_t i = 0; i<numDriveMotors; i++)
  {
    leftDriveMotors[i].spinFor(rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);

    if(i == numDriveMotors-1) //this is the last motor in the array and needs to be blocking so we complete the move
    {
      rightDriveMotors[i].spinFor(-1*rotations, rotationUnits::rev, velocity, velocityUnits::pct, true);
    }
    else
    {
      rightDriveMotors[i].spinFor(-1*rotations, rotationUnits::rev, velocity, velocityUnits::pct, false);
    }
  }
#endif
}

void updateDriveMotors(int leftDriveVal, int rightDriveVal)
{
  for(uint8_t i = 0; i < numDriveMotors; i++)
  {
    leftDriveMotors[i].spin(directionType::fwd, leftDriveVal, percentUnits::pct);
    rightDriveMotors[i].spin(directionType::fwd, rightDriveVal, percentUnits::pct);
  }
}

void updateDriveMotorVolts(int leftVoltage, int rightVoltage)
{
  for(uint8_t i = 0; i < numDriveMotors; i++)
  {
    leftDriveMotors[i].spin(directionType::fwd, leftVoltage, voltageUnits::volt);
    rightDriveMotors[i].spin(directionType::fwd, rightVoltage, voltageUnits::volt);
  }  
}

void setDriveBrake(brakeType brake_type)
{
  for(uint8_t i = 0; i < numDriveMotors; i++)
  {
    leftDriveMotors[i].setBrake(brake_type);
    rightDriveMotors[i].setBrake(brake_type);
  } 
}

float getMotorAvgRotations(vector<motor> motorGroup)
{
  float averageRotations = 0;
  for(uint8_t i = 0; i < numDriveMotors; i++)
  {
    averageRotations += motorGroup[i].position(rotationUnits::rev);
  }
  averageRotations /= numDriveMotors;
  return averageRotations;
}

void resetDriveRotations(void)
{
for(uint8_t i = 0; i < numDriveMotors; i++)
  {
    leftDriveMotors[i].resetPosition();
    rightDriveMotors[i].resetPosition();
  }
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
