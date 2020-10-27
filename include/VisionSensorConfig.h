/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BLUE_BALL       = signature (1, -3435, -2457, -2946, 8277, 11503, 9890, 3.9, 0);
vex::vision::signature BLUE_BALL_DARK  = signature (4, -2637, -1965, -2302, 8827, 10923, 9874, 4, 0);
vex::vision::signature RED_BALL        = signature (2, 7427, 11021, 9224, -1737, -1033, -1386, 2.9, 0);
vex::vision::signature RED_BALL_DARK   = signature (3, 3659, 7073, 5366, 1699, 3181, 2440, 2.1, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision visionDummy2  = vex::vision (PORT22, 50, BLUE_BALL, RED_BALL, BLUE_BALL_DARK, RED_BALL_DARK);
/*vex-vision-config:end*/