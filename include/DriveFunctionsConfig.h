/********************************************************************
 *    DriveFunctions.h
 * this is where all the config variables for a robot are located. 
 * Change them to match your robot
 * these variables are used in DriveFunctions.cpp
 *******************************************************************/

#ifndef DRIVEFUNCTIONSCONFIG_H
#define DRIVEFUNCTIONSCONFIG_H

#include "vex.h"
using namespace vex;

#include <math.h>
#include "vex_thread.h"
#include "PID.h"

//function declarations
void moveLinear(float distance, int velocity, uint32_t timeOut);
void moveRotate(int16_t degrees, int velocity, uint32_t timeOut);
void moveStop(void);
void userDrive(void);

//this is where all the config variables for a robot are located. Change them to match your robot
//these variables are used in DriveFunctions.cpp

/*    Chassis Config 
 * uncomment the chassis type here
 * x drive assumes 4 drive motors
 */
#define CHASSIS_4_MOTOR_INLINE
//#define CHASSIS_2_MOTOR_INLINE
//#define CHASSIS_X_DRIVE

/*    Drive Motor Ports
 * Enter the ports used for 
 *  drive motors
 * Perspective is from rear of robot
 *  looking forward
 */
#ifdef CHASSIS_4_MOTOR_INLINE
  extern motor FrontLeft;
  extern motor FrontRight;
  extern motor BackLeft;
  extern motor BackRight;

  #define FrontLeftPort     PORT10
  #define FrontRightPort    PORT20
  #define BackLeftPort      PORT1
  #define BackRightPort     PORT5

#elif defined(CHASSIS_2_MOTOR_INLINE)
  extern motor DriveLeft;
  extern motor DriveRight;

  #define DriveLeftPort     PORT6
  #define DriveRightPort    PORT8
#endif

/*    Control Scheme
 * Define your control scheme and joystick
 *  axes here
 */
 #define ARCADE_CONTROL
 //#define TANK_CONTROL

#ifdef ARCADE_CONTROL
  #define HORIZONTALAXIS Axis4
  #define VERTICALAXIS Axis3
#elif def TANK_CONTROL
  #define LEFTAXIS Axis3
  #define RIGHTAXIS Axis2
#endif

/*    Chassis Dimensions
 * Enter the length and width 
 * (from wheel center to wheel center) of 
 * your chassis here in inches
 */
 #define CHASSIS_WIDTH    12.00f
 #define CHASSIS_LENGTH   9.00f

#define HYPOTENUSE sqrt(pow(CHASSIS_WIDTH,2) + pow(CHASSIS_LENGTH,2))
#define RADIUS  HYPOTENUSE/2.0f
#define CIRCUMFERENCE HYPOTENUSE * M_PI

/*     Wheel Size 
 * uncomment the size of drive wheel you are using
 */
#define WHEEL_SIZE    4.0f
//#define WHEEL_SIZE  2.75f

/*     Gear Ratio 
* Enter the gear ratio of your drive
* this is not the same as the motor gear set
* Leave as 1 if you have a direct drive
*/
#define GEAR_RATIO 1.0f

/*     ROTATION_FACTOR 
 * this is a constant ratio
 *  accounting for gear ration and wheel
 *  size
 */
#define ROTATION_FACTOR M_PI * WHEEL_SIZE * GEAR_RATIO

/*    Motor Gearset 
* uncomment the color of gearset you
* are using in the motors
*/
#define RED   ratio36_1
#define BLUE  ratio6_1
#define GREEN ratio18_1

//#define GEAR_SET    RED
//#define GEAR_SET    BLUE
#define GEAR_SET    GREEN

#define PID
//#define GYRO

#ifdef PID
  void setLinGains(double kP, double kI, double kD, double slewRate, int minDT);
  void setRotGains(double kP, double kI, double kD, double slewRate, int minDT);
#endif

#ifdef GYRO
  extern inertial myGyro;
  #define GYRO_PORT PORT19
#endif

#endif