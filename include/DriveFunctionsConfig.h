/********************************************************************
 *    DriveFunctions.h
 * this is where all the config variables for a robot are located. Change them to match your robot
 * these variables are used in DriveFunctions.cpp
 *******************************************************************/

#ifndef DRIVEFUNCTIONSCONFIG_H
#define DRIVEFUNCTIONSCONFIG_H
using namespace vex;

#include <math.h>

//function declarations
void moveLinear(float distance, int velocity);

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

  #define FrontLeftPort     PORT1
  #define FrontRightPort    PORT2
  #define RearLeftPort      PORT3
  #define RearRightPort     PORT4

#elif defined(CHASSIS_2_MOTOR_INLINE)
  extern motor DriveLeft;
  extern motor DriveRight;

  #define DriveLeftPort     PORT1
  #define DriveRightPort    PORT2
#endif

/*    Chassis Dimensions
 * Enter the length and width of 
 * your chassis here in inches
 */
 #define CHASSIS_WIDTH    12.0f
 #define CHASSIS_LENGTH   12.0f

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
#define RED   ratio18_1
#define BLUE  ratio_36_1
#define GREEN ratio6_1

#define GEAR_SET    RED
//#define GEAR_SET    BLUE
//#define GEAR_SET    GREEN

#endif