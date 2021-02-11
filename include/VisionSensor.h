/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BLUE_BALLS = vex::vision::signature (1, -3121, -2033, -2577, 7927, 12333, 10130, 3.1, 0);
vex::vision::signature RED_BALLS = vex::vision::signature (2, 7141, 9709, 8425, -805, 1, -402, 1.8, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision VisionSensor = vex::vision (vex::PORT1, 50, BLUE_BALLS, RED_BALLS, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/