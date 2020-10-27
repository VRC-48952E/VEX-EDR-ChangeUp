#include "vex.h"
#include "helperfunctions.h"
#include "autonomous.h"
#include "robot-config.h"
#include "math.h"

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (true) {
    //Define controlleraxis for the drivetrain 
    double axisSpin            = controllerPrimary.Axis1.position();
    double axisForwardBackward = controllerPrimary.Axis2.position();
    double axisLeftRight       = controllerPrimary.Axis4.position();

    // increase sensitivity by taking the axis values to the power of 3 or 5 (using values between 0.0 and 1.0)
    // see: https://www.vexforum.com/t/controller-axis-sensitivity/56618
    axisLeftRight = pow((axisLeftRight/100.0), 3) * 100;
    axisForwardBackward = pow((axisForwardBackward/100.0), 3) * 100;
    axisSpin = pow((axisSpin/100.0), 3) * 100;


    // spin robot
    /*
    LeftFrontMotor.spin(directionType::fwd, axisSpin, velocityUnits::pct);
    LeftRearMotor.spin(directionType::fwd, axisSpin, velocityUnits::pct);
    RightFrontMotor.spin(directionType::fwd, axisSpin, velocityUnits::pct);
    RightRearMotor.spin(directionType::fwd, axisSpin, velocityUnits::pct);
    */

    // drive forward
    /*
    LeftFrontMotor.spin(directionType::fwd, axisForwardBackward, velocityUnits::pct);
    LeftRearMotor.spin(directionType::fwd, axisForwardBackward, velocityUnits::pct);
    RightFrontMotor.spin(directionType::fwd, axisForwardBackward*-1.0, velocityUnits::pct);
    RightRearMotor.spin(directionType::fwd, axisForwardBackward*-1.0, velocityUnits::pct);
    */

    // drive sideways
    /*
    LeftFrontMotor.spin(directionType::fwd, axisLeftRight, velocityUnits::pct);
    LeftRearMotor.spin(directionType::fwd, axisLeftRight*-1.0, velocityUnits::pct);
    RightFrontMotor.spin(directionType::fwd, axisLeftRight, velocityUnits::pct);
    RightRearMotor.spin(directionType::fwd, axisLeftRight*-1.0, velocityUnits::pct);
      */
    
    
    // drive combined / overlay all single directions from above
    leftFrontMotor.spin(directionType::fwd,  axisSpin + axisForwardBackward + axisLeftRight, velocityUnits::pct);
    leftRearMotor.spin(directionType::fwd,   axisSpin + axisForwardBackward - axisLeftRight, velocityUnits::pct);
    rightFrontMotor.spin(directionType::fwd, axisSpin - axisForwardBackward + axisLeftRight, velocityUnits::pct);
    rightRearMotor.spin(directionType::fwd,  axisSpin - axisForwardBackward - axisLeftRight, velocityUnits::pct);
  

    if(controllerPrimary.ButtonX.pressing()) {
    }

    //opens filter (sort a ball out)
    if(controllerPrimary.ButtonDown.pressing()) {
      filterMoveOpen();
    }
    //closes filter
    else if(controllerPrimary.ButtonUp.pressing()) {
      filterMoveClosed();
    }
    //make intakemotor spin (intake a ball)
    if(controllerPrimary.ButtonL1.pressing()) {
      intakeMotor.spin(directionType::fwd, 75, percentUnits::pct);
    }
    else {
      intakeMotor.stop(brakeType::coast);
    }
    //make transportmotor spin (transport a ball)
    if(controllerPrimary.ButtonR1.pressing()) {
      transportMotor.spin(directionType::fwd, 75, percentUnits::pct);
    }
    else {
      transportMotor.stop(brakeType::coast);
    }
    //make outputmotor spin (shoot a ball out)
    if(controllerPrimary.ButtonR2.pressing()) {
      outputMotor.spin(directionType::fwd, 75, percentUnits::pct);
    }
    else {
      outputMotor.stop(brakeType::coast);
    }
    //make every motor (for ball transportion) spin (used for testing and not really in game)
    if(controllerPrimary.ButtonL2.pressing()){
      intakeMotor.spin(directionType::fwd, 100, percentUnits::pct);
      transportMotor.spin(directionType::fwd, 100, percentUnits::pct);
      outputMotor.spin(directionType::fwd, 100, percentUnits::pct);
    }
    else {
      intakeMotor.stop(brakeType::coast);
      transportMotor.stop(brakeType::coast);
      outputMotor.stop(brakeType::coast);
    }
    //use visionsensor to center on a blue ball
    if(controllerPrimary.ButtonA.pressing()) {
      centerOnBall(BLUE_BALL);
    }
    //uses widh of the ball to keep the robot at a spezifik distance to the ball
    if(controllerPrimary.ButtonB.pressing()) {
      approachBallUntilWidthReached(BLUE_BALL, 120);
    }
    //use vision sensor to follow a ball (uses position and width of a ball)
    if(controllerPrimary.ButtonY.pressing()) {
      followBall(BLUE_BALL);
    }

    // prevent wasted resources.
    vex::task::sleep(20); // Sleep the task for a short amount of time 
  }
}