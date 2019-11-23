//crucial that you include the header file that lists the autons

#include "Autons.h"

void Auton2(){
  ClawMotor.spinFor(1,turns, false);
  Tester1.spinFor(3,turns);
}