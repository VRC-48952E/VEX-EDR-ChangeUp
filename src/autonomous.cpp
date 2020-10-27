/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/

#include "vex.h"
#include "helperfunctions.h"
#include "robot-config.h"

using namespace vex;

void startSequence() {
  releaseUpperRail();
  wait(1200, timeUnits::msec);
  getBallInShootingPosition(true, false);
}

void RunAutonomous15s(bool redAlliance = true, bool startRight = true) {
  // toDo: implement code
  
  if (redAlliance) { // run 15s autonomous as member of Red Alliance
  }
  else { // run 15s autonomous as member of Blue Alliance
  }

}

void RunAutonomous60s() {
  // during program display detected balls on brain; runs in a separate thread
  vex::thread(drawDetectedBallsWithVisionSensorsInfinite).detach();

  // unfold upper ramp and put preload in shooting position
  vex::thread(startSequence).detach();

  // run intake for ball infront of robot at starting position
  vex::thread(intakeBallUntilFilterPosition).detach();

  // while intake runs for 2nd ball drive forward to a distance where the 3rd red ball is to the left of the robot 
  driveRearUntilDistance(50);
  // move closer to side the wall with the next red ball 
  driveRightUntilDistance(40, 5);
  driveRearUntilDistance(70,5);

  // turn right to 90 degrees to collect 3rd red ball
  turnRobotToAngle(90);
  intakeMotor.stop(brakeType::coast);
  transportMotor.stop(brakeType::coast);
  // align to red ball w/ help of vision sensors (in case the distance wasn't precise enough or the initial ball position is off)
  centerOnBall(RED_BALL,"sideways", 5);

  // run intake for 3rd ball and move forwards the field side
  transportMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  intakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  driveFrontUntilDistance(190, 5, distanceUnits::mm, 10);
  
  // wait a little bit to allow ball to be intaken; then move backwards
  wait(1500, timeUnits::msec);

  // drive to corner and turn left by 45 degrees (to angle 135 degrees)
  driveFrontUntilDistance(275, 5, distanceUnits::mm);
  driveRightUntilDistance(270, 5, distanceUnits::mm);
  // run intake mechanism until 3rd ball is at filter position
  vex::thread(intakeBallUntilFilterPosition).detach();
  
  turnRobotToAngle(135);
  // use vision sensor to align to blue ball in tower (for correcting the position - if needed)
  centerOnBall(BLUE_BALL,"sideways", 5);
  
  // approach tower a bit more to shoot ball
  driveRobotForward(directionType::fwd, 10);
  wait(900, timeUnits::msec);
  stopDriveTrain();

  // shoot 1st red ball to corner twoer
  outputMotor.rotateFor(directionType::fwd, 8, rotationUnits::rev, 100, velocityUnits::pct);

  // drive a bit backwards and turn to 180 degrees heading
  driveRobotForward(directionType::rev, 20);
  wait(1000, timeUnits::msec);
  stopDriveTrain();
  turnRobotToAngle(180);

  // meanwhile get next ball in shooting position
  vex::thread(getBallInShootingPosition).detach();

  // drive backwards to a distance where the tower 2 (side middle) when driving sidewards
  driveFrontUntilDistance(480, 5, distanceUnits::mm);
  wait(800, timeUnits::msec);
  driveLeftUntilDistance(150);
  
  // when near (in front of the tower) use vision sensor to center on blue ball (in the tower)
  centerOnBall(BLUE_BALL,"sideways", 5);
  // drive forward (15cm)
  driveRobotFor(15, directionType::fwd, 10);
  // shoot 2nd red ball to middle tower at the side
  outputMotor.rotateFor(directionType::fwd, 10, rotationUnits::rev, 100, velocityUnits::pct);
  
  // get next ball in shooting position
  vex::thread(getBallInShootingPosition).detach();

  // drive backwards (15cm) and turn 180 degrees (to heading 0 degrees)
  driveRobotFor(15, directionType::rev, 10);
  turnRobotToAngle(0);

  // align to red ball w/ help of vision sensors (to intake 4th ball in front of center tower)
  centerOnBall(RED_BALL,"sideways", 5);
    
  // run intake motors and drive forward to intake 4th red ball (in front of center tower)
  transportMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  intakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  driveRobotFor(30, directionType::fwd, 10);
  
  // run intake motors until red ball reaches filter position
  vex::thread(intakeBallUntilFilterPosition).detach();
  
  // position robot with help of distance sensors (to two sides)
  driveRightUntilDistance(1950, 10, distanceUnits::mm);
  driveRearUntilDistance(1000, 10, distanceUnits::mm);
  driveRightUntilDistance(1600, 10, distanceUnits::mm);

  // drive 25cm forwards to push blue ball out of middle tower
  driveRobotFor(25, directionType::fwd, 20);
  // use vision sensors to turn to blue ball = center to middle tower
  centerOnBall(BLUE_BALL,"turn", 5);
  // shoot 3rd red ball to center tower
  outputMotor.rotateFor(directionType::fwd, 12, rotationUnits::rev, 100, velocityUnits::pct);
  // move robot back a bit 
  driveRobotFor(15, directionType::rev, 10);

  stopDriveTrain();
}

// triggered by field controller (judges)
void autonomous( void ) {
  RunAutonomous60s();
}