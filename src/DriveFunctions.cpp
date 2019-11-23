//crucial that you include the header file that lists the functions

#include "Functions.h"

void oneMotor(int speed, int spins){
  Tester1.setVelocity(speed,percent);
Tester1.spinFor(spins,turns);
Tester1.setStopping(brake);
}


void twoMotors(int speed, int spins){
  Tester1.setVelocity(speed,percent);
  ClawMotor.setVelocity(speed,percent);

Tester1.setStopping(brake);
ClawMotor.setStopping(brake);

Tester1.spinFor(spins,turns,false);
ClawMotor.spinFor(spins,turns);

Tester1.stop();
ClawMotor.stop();
}