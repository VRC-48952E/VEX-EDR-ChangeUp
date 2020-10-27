#include "vex.h"
#include "robot-config.h"
#include "stdlib.h"
#include <math.h>       /* round, floor, ceil, trunc */

// function to easily convert numbers to strings (https://stackoverflow.com/questions/3513173/converting-ostream-into-standard-string)
#include <sstream>
#include <string>
template <typename T> std::string toString(T value) {
  std::ostringstream os;
  os << value;
  return os.str();
}

/*
return random numbers as characters (e.g. to prepend / generate a specific filename) needs: <cstdlib>
see: http://www.cplusplus.com/reference/cstdlib/rand/ */
#include <cstdlib>
std::string getRandomStr(int numChars) {
  std::string mystr = "";
  for (int i = 0; i < numChars; i++) {
    mystr.append(toString(rand() % 9));
  }
  return mystr;
}

/*
writing debug information to a file (on Brain's SD Card Slot)
function openDebugFile    -> try to open a file for writing; return true if operation was successful
function closeDebugFile   -> closes the open file
function writeToDebugFile -> writes a line to the open file; if file is not open: open a file, write message, close the file
needs: <fstream>
see: http://www.cplusplus.com/reference/fstream/ofstream/open/
*/
#include <fstream>
std::ofstream ofs;
bool openDebugFile(std::string fileName = "BestGhosts_Debug_File.txt") {
  // check if SD Card is present
  if (Brain.SDcard.isInserted()) {
    // tr to open file for writing (std::ofstream::app means open and write at the end of the file (append))
    ofs.open(fileName, std::ofstream::app);
    // check if file could be opened (for writing)
    if (ofs.is_open()) {
      return true;
    }
    else {
      return true;
    }
  } else {
    return false;
  }
}
void closeDebugFile() { ofs.close(); }
void writeToDebugFile(std::string debugMessage) {
  // checks if file is already open
  bool wasOpen = ofs.is_open();
  // if file is not open then open the file for writing
  if (wasOpen != true) {
    openDebugFile();
  }

  // write the message to the open file
  ofs << debugMessage;
  // write newline characters to the end of the message (above)
  ofs << "\r\n";

  // if the file was not open (see above) and needed to be opened: close the open file
  if (wasOpen != true) {
    closeDebugFile();
  }
}

// prints a message to the Brain Screem and adds a newline
void printDebugMessageOnBrain(std::string debugMessage) {
  Brain.Screen.newLine();
  Brain.Screen.print(debugMessage.c_str());
}

// prints a message to the Controller and adds a newline
void printDebugMessageOnController(std::string debugMessage, vex::controller controllerToPrintTo= controllerPrimary) {
  controllerToPrintTo.Screen.newLine();
  controllerToPrintTo.Screen.print(debugMessage.c_str());
}

/*
 * this function calculates the active time of motors to travel a given distance
 * it uses values from calibration runs
 *
 * distance = time * a + b	(offset by 300ms due to start-to-move-delay)
 * speed   10%     20%     30%     40%
 * a	   0,148	 0,295 	 0,447	 0,596
 * b	   7,500	19,500	21,750	27,750
 * 
 * time = (target-distance - b) / a
 */
double getTraveltimeForDistance (int travelDistance, int speed = 20) {
  double traveltime = 0.0;
  switch (speed) {
    case 10:
      traveltime = (travelDistance - 7.5) / 0.148;
      break;
    case 20:
      traveltime = (travelDistance - 19.5) / 0.295;
      break;
    case 30:
      traveltime = (travelDistance - 21.75) / 0.447;
      break;
    case 40:
      traveltime = (travelDistance - 27.75) / 0.596;
      break;
  }
  return traveltime + 800;
}

/*
 * function is used to stop all 4 drive train motorsof the  robot (x-drive)
 * one can choose when calling the function how the motors should stop; default is brakeType::brake
 * in order to reduce a motor temperature issue the motors are set to keep loose at the end
 */
void stopDriveTrain(brakeType myBrakeType = brakeType::brake) {
  leftFrontMotor.stop(myBrakeType);
  leftRearMotor.stop(myBrakeType);
  rightFrontMotor.stop(myBrakeType);
  rightRearMotor.stop(myBrakeType);
  // keep motors away from holding the whole time (brakeType::brake or ::hold) -> temperature issue
  myBrakeType = brakeType::coast;
  leftFrontMotor.stop(myBrakeType);
  leftRearMotor.stop(myBrakeType);
  rightFrontMotor.stop(myBrakeType);
  rightRearMotor.stop(myBrakeType);
}

/*
 * function is used to turn the robot (x-drive)
 * needs a stop command in order to not turn for all times -> stopDriveTrain();
 */
void turnRobot(vex::directionType spinDirection, int spinVelocity) {
  leftFrontMotor.spin(spinDirection, spinVelocity, velocityUnits::pct);
  leftRearMotor.spin(spinDirection, spinVelocity, velocityUnits::pct);
  rightFrontMotor.spin(spinDirection, spinVelocity, velocityUnits::pct);
  rightRearMotor.spin(spinDirection, spinVelocity, velocityUnits::pct);
}

/*
function is use to turn to a specified angle / heading
turn speed is adjusted to how far the robot is away from target heading (slower when getting closer to target value) 
tolerance for turn precision are 3 degrees by default
*/
void turnRobotToAngle(int targetAngle, int turnSpeed = 20, int tolerance = 3) {
  int error = 0; // represents difference value between current angle and target angle
  int actTurnSpeed = turnSpeed; // actual turn speed (when closer to target angle with reduced turn speed)

  // looping until difference is below tolerance threshild (error < tolerance)
  error = (int)gyroSensor.rotation(rotationUnits::deg) - targetAngle;
  while (abs(error) > tolerance) { // use abs(error) as absolute (positive) difference as it could be negative (far off) and below tolerance
    // turn slower when aproaching the target angle (but at least 10% speed)
    actTurnSpeed = std::min(abs(error), turnSpeed);
    actTurnSpeed = std::max(abs(error), 10);

    if (error < 0) {
      // turn right
      //turnRobot(vex::directionType::rev, std::max(error, 10));
      turnRobot(vex::directionType::fwd, actTurnSpeed);
    } else {
      // turn left
      turnRobot(vex::directionType::rev, actTurnSpeed);
    }

    // re-calculate error for while loop condition (above)
    error = (int)gyroSensor.rotation(rotationUnits::deg) - targetAngle;
  } 

  // stop all motors
  stopDriveTrain();
}

/*
function is used to turn by a certain degree; it uses the turnRobotToAngle
function */
void turnRobotByAngle(int angleToRotate) {
  turnRobotToAngle(gyroSensor.rotation(rotationUnits::deg) + angleToRotate);
}

void driveRobotForward(vex::directionType spinDirection, int spinVelocity) {
  leftFrontMotor.spin(spinDirection, spinVelocity, velocityUnits::pct);
  leftRearMotor.spin(spinDirection, spinVelocity, velocityUnits::pct);
  rightFrontMotor.spin(spinDirection, spinVelocity * -1.0, velocityUnits::pct);
  rightRearMotor.spin(spinDirection, spinVelocity * -1.0, velocityUnits::pct);
}

void driveRobotSideways(vex::directionType spinDirection, int spinVelocity) {
  leftFrontMotor.spin(spinDirection, spinVelocity, velocityUnits::pct);
  leftRearMotor.spin(spinDirection, spinVelocity * -1.0, velocityUnits::pct);
  rightFrontMotor.spin(spinDirection, spinVelocity, velocityUnits::pct);
  rightRearMotor.spin(spinDirection, spinVelocity * -1.0, velocityUnits::pct);
}

/*
 * function is used to drive for a specified distance
 * it uses the values from calibration
 * driveSpeed can be specified as percentage (more than 50% is not very precise!)
 */
void driveRobotFor(int distanceToTravel, directionType mydirection = directionType::fwd, int driveSpeed = 20) {  
  double travelTime = getTraveltimeForDistance(distanceToTravel, driveSpeed);
  printDebugMessageOnController("traveltime: " + toString(travelTime));
  driveRobotForward(mydirection, driveSpeed);
  
  // drive calculated time
  wait(travelTime, timeUnits::msec);

  // stop all motors
  stopDriveTrain();
}

/*
 * function is used to drive forwards until the front distance sensor measurement equals a given value (distance)
 * when calling the function one can choose precision by providing a tolerance value: [distance - tolerance, distance + tolerance]
 */
void driveFrontUntilDistance(int targetDistance, int tolerance = 5, distanceUnits mydistanceUnit = distanceUnits::cm, int driveSpeedMax = 40) {
  int driveSpeedMin = 10;
  int driveSpeed = driveSpeedMin;
  double currentDistanceFront = distanceFront.objectDistance(mydistanceUnit);
  while(abs((int)roundf(currentDistanceFront) - targetDistance) > tolerance) {
    // set drive speed between 10% and at max. 50% to adapt on how far away the target is
    driveSpeed = (int)(abs((int)roundf(currentDistanceFront) - targetDistance) / targetDistance) * 100 / 2;
    // apply upper and lower boundary
    driveSpeed = std::max(driveSpeed, driveSpeedMin);
    driveSpeed = std::min(driveSpeed, driveSpeedMax);

    // determine whether to drive closer (forward) or to move backwards
    if ((int)roundf(currentDistanceFront) - targetDistance > 0) {
      driveRobotForward(vex::directionType::fwd, driveSpeed);
    }
    else {
      driveRobotForward(vex::directionType::rev, driveSpeed); 
    }

    // measure current distance for next cycle
    currentDistanceFront = distanceFront.objectDistance(mydistanceUnit);
    wait(50, timeUnits::msec);
  }

  stopDriveTrain(brakeType::brake);
}

/*
 * function is used to drive backwards until the rear sonic sensor measurement is below a given value (distance)
 * when calling the function one can choose precision by providing tolerance value: [distance - tolerance, distance + tolerance]
 */
void driveRearUntilDistance(int targetDistance, int tolerance = 5, distanceUnits mydistanceUnit = distanceUnits::cm) {
  int driveSpeed = 30;
  double currentDistanceRear = sonarRear.distance(mydistanceUnit);
  while(abs((int)roundf(currentDistanceRear) - targetDistance) > tolerance) {
    if ((int)roundf(currentDistanceRear) - targetDistance < 0) {
      driveRobotForward(vex::directionType::fwd, driveSpeed);
    }
    else {
      driveRobotForward(vex::directionType::rev, driveSpeed); 
    }
    currentDistanceRear = sonarRear.distance(mydistanceUnit);
  }
  stopDriveTrain(brakeType::brake);
}

/*
 * function is used to drive sideways until the right sonic sensor measurement equals a given value (distance)
 * when calling the function one can choose precision by providing tolerance value: [distance - tolerance, distance + tolerance]
 */
void driveRightUntilDistance(int targetDistance, int tolerance = 5, distanceUnits mydistanceUnit = distanceUnits::cm) {
  int driveSpeedMin = 10;
  int driveSpeedMax = 50;
  int driveSpeed = driveSpeedMin;
  
  double currentDistanceRight = sonarRight.distance(mydistanceUnit);
  while(abs((int)roundf(currentDistanceRight) - targetDistance) > tolerance) {
    // set drive speed between 10% and at max. 50% to adapt on how far away the target is
    driveSpeed = (int)(abs((int)roundf(currentDistanceRight) - targetDistance) / targetDistance) * 100 / 2;
    // apply upper and lower boundary
    driveSpeed = std::max(driveSpeed, driveSpeedMin);
    driveSpeed = std::min(driveSpeed, driveSpeedMax);

    if ((int)roundf(currentDistanceRight) - targetDistance < 0) {
      driveRobotSideways(vex::directionType::rev, driveSpeed);
    }
    else {
      driveRobotSideways(vex::directionType::fwd, driveSpeed);
    }
    currentDistanceRight = sonarRight.distance(mydistanceUnit);
  }
  stopDriveTrain(brakeType::brake);
}

/*
 * function is used to drive sideways until the left sonic sensor measurement equals a given value (distance)
 * when calling the function one can choose precision by providing tolerance value: [distance - tolerance, distance + tolerance]
 */
void driveLeftUntilDistance(int targetDistance, int tolerance = 5, distanceUnits mydistanceUnit = distanceUnits::cm) {
  int driveSpeed = 30;
  double currentDistanceLeft = sonarLeft.distance(mydistanceUnit);
  while(abs((int)roundf(currentDistanceLeft) - targetDistance) > tolerance) {
    if ((int)roundf(currentDistanceLeft) - targetDistance < 0) {
      driveRobotSideways(vex::directionType::fwd, driveSpeed);
    }
    else {
      driveRobotSideways(vex::directionType::rev, driveSpeed);
    }
    currentDistanceLeft = sonarLeft.distance(mydistanceUnit);
  }
  stopDriveTrain(brakeType::brake);
}

// function opens the back of the robot to release a ball (of oponent color)
void filterMoveOpen() {
  double targetPosition = -0.85;
  filterMotor.spinToPosition(targetPosition, rotationUnits::rev, 30, velocityUnits::pct, false);
  transportMotor.spinFor(directionType::rev, 1.0, timeUnits::sec);
  /*
  // spin motor until it reaches a physical block (torgue gets high)
  while(filterMotor.torque(torqueUnits::Nm) < 0.5){
    filterMotor.rotateFor(directionType::rev, 65, rotationUnits::deg ,30, velocityUnits::pct);
  }
  filterMotor.stop(brakeType::brake);
  */
}

// function closes the back of the robot to allow normal ball flow
void filterMoveClosed() {
  double targetPosition = 0.0;
  filterMotor.spinToPosition(targetPosition, rotationUnits::rev, 80, velocityUnits::pct, false);
  /*
  // spin motor until it reaches a physical block
  while(filterMotor.torque(torqueUnits::Nm) < 0.4){
    filterMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  }
  filterMotor.stop(brakeType::brake);
  */
}   

/*
functon uses the two vision sensors to align robot in that way that both sensors "see" the ball at the same position
one can choose if the robot should turn or to drive sideways to center on ball  
*/
void centerOnBall(signature vision_signature, std::string alignmentMethod = "sideways", int tolerance = 10, int alignmentSpeed = 10) {
  // take snapshot on both vision sensors with the given signature (e.g. blue ball signature)
  visionRight.takeSnapshot(vision_signature);
  visionLeft.takeSnapshot(vision_signature);

  // minimal ball size (stable detection possible)
  int minimalDetectedBallWidth = 30;

  // if both sensors "see" an object (largestObject.exists) then try to align to the corresponding ball
  if (
        visionLeft.largestObject.exists && 
        visionRight.largestObject.exists && 
        visionLeft.largestObject.width > minimalDetectedBallWidth &&
        visionRight.largestObject.width > minimalDetectedBallWidth
      ) {
    vex::directionType leftRightDirection = directionType::fwd;
    while (
            (visionLeft.largestObject.exists && visionRight.largestObject.exists) &&
            (
              (visionRight.largestObject.centerX + visionLeft.largestObject.centerX) / 2 > 158 + tolerance ||
              (visionRight.largestObject.centerX + visionLeft.largestObject.centerX) / 2 < 158 - tolerance
            )
          ) {
      // check if the ball is more right (then turn in direction right (fwd)) or more left (vice versa)
      if ((visionRight.largestObject.centerX + visionLeft.largestObject.centerX) / 2 < 158) {
        leftRightDirection = directionType::rev;
      } else {
        leftRightDirection = directionType::fwd;
      }
      // switch for method to align by turning or driving sideways
      if (alignmentMethod.compare("sideways") == 0) {
        // use sideways alignment (keep heading)
        driveRobotSideways(leftRightDirection, alignmentSpeed);
      }
      else {
        // use turning the robot to center on ball 
        turnRobot(leftRightDirection, alignmentSpeed);
      }

      // wait some time for next measure cycle
      vex::wait(100, timeUnits::msec);
      stopDriveTrain();
      visionRight.takeSnapshot(vision_signature);
      visionLeft.takeSnapshot(vision_signature);
    }
  }
}

/*
function uses the two vision sensors to keep robot to a certain distance by measuring the detected width
both sensors need to "see" the ball
alignment on distance with variable widthToAlign
*/
void approachBallUntilWidthReached(signature vision_signature, int widthToAlign, int tolerance = 10, int alignmentSpeed = 10) {
  // take snapshot on both vision sensors with the given signature (e.g. blue ball signature)
  visionRight.takeSnapshot(vision_signature);
  visionLeft.takeSnapshot(vision_signature);

  // minimal ball size (stable detection possible)
  int minimalDetectedBallWidth = 30;

  // if both sensors "see" an object (largestObject.exists) then try to align to the corresponding ball
  if (
        visionLeft.largestObject.exists && 
        visionRight.largestObject.exists && 
        visionLeft.largestObject.width > minimalDetectedBallWidth &&
        visionRight.largestObject.width > minimalDetectedBallWidth
      ) {
    // keep driving while measured width is larger or smaller than widthToAlign (+/- tolerance)
    // measured width: average of both sensors -> (width sensor left + width sensor right) divided by 2 
    while (
            (visionLeft.largestObject.exists && visionRight.largestObject.exists) &&
            (
              (visionRight.largestObject.width + visionLeft.largestObject.width) / 2 > widthToAlign + tolerance ||
              (visionRight.largestObject.width + visionLeft.largestObject.width) / 2 < widthToAlign - tolerance
            )
          ) {
      // if measured width is smaller then drive towars the ball (forward)
      if ((visionRight.largestObject.width + visionLeft.largestObject.width) / 2 < widthToAlign) {
        driveRobotForward(directionType::fwd, alignmentSpeed);
      } 
      else { // else if measured width is larger than target width (widthToAlign) then drive backwards
        driveRobotForward(directionType::rev, alignmentSpeed);
      }

      // wait some time for next measure cycle
      vex::wait(100, timeUnits::msec);
      stopDriveTrain();
      visionRight.takeSnapshot(vision_signature);
      visionLeft.takeSnapshot(vision_signature);
    }
  }
}

void followBall(signature vision_signature) {
  int minimalDetectedBallWidth = 30;
  visionRight.takeSnapshot(vision_signature);
  visionLeft.takeSnapshot(vision_signature);
  // run only if both vision sensors "see" a ball of a certain size or bigger
  // (minimalDetectedBallWidth)
  if (visionLeft.largestObject.exists && visionRight.largestObject.exists &&
      visionLeft.largestObject.width > minimalDetectedBallWidth &&
      visionRight.largestObject.width > minimalDetectedBallWidth) {
    // first: center robot on ball
    centerOnBall(vision_signature);
    // second: drive to a defined distance (largestObject.width)
    approachBallUntilWidthReached(vision_signature, 120);
  }
}

/*
 * the expanded upper ball rail exceeds the maximum allowed hight   
 * this function uses slow backward rotation to release the upper rail for the ball transport upon match start
 */
void releaseUpperRail() {
  outputMotor.rotateFor(directionType::rev, 2.0, rotationUnits::rev, 5, velocityUnits::pct, false);
}

/*
 * the expanded intake arms exceed the maximum allowed robot length   
 * this function uses rotation of intke motor to release the arms and bring them to front position
 */
void releaseSideArms() {
  intakeMotor.rotateFor(5, timeUnits::sec, 100, velocityUnits::pct);
}

/*
 * runs intake motor until ball is in filter position
 */
void intakeBallUntilFilterPosition() {
  double maxRunTime = 5.0; // in seconds
  double startTime = Brain.Timer.value();

  intakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  while(limitSwitchFilterPosition.pressing() == 0 && Brain.Timer.value() < (startTime + maxRunTime)) {
    wait(50, timeUnits::msec);
  }
  wait(800, timeUnits::msec);
  intakeMotor.stop(brakeType::coast);
}

/*
 * runs transport & output chains until next ball is in shooting position
 */
void getBallInShootingPosition(bool controlTransport = true, bool controlIntake = false) {
  double maxRunTime = 5.0; // in seconds
  double startTime = Brain.Timer.value();

  // while no ball triggers limitswitch run motors that transport the ball; afterwards the motors are stopped
  while(limitSwitchShootingPosition.pressing() == 0 && Brain.Timer.value() < (startTime + maxRunTime)) {
    outputMotor.spin(directionType::fwd, 100, percentUnits::pct);
    if(controlTransport) {
      transportMotor.spin(directionType::fwd, 100, percentUnits::pct);
    }
    if(controlIntake) {
      intakeMotor.spin(directionType::fwd, 100, percentUnits::pct);
    }
    wait(50, msec);
  }
  outputMotor.stop(brakeType::coast);
  if(controlTransport) {
    transportMotor.stop(brakeType::coast);
  }
  if(controlIntake) {
    intakeMotor.stop(brakeType::coast);
  }
}

// for a detached call no arguments are allowed; so we overload the function call
// tutorial: http://www.cplusplus.com/doc/tutorial/functions2/ 
void getBallInShootingPosition() {
  getBallInShootingPosition(true, false);
}

/*
 * this function slowly brings filter motor to start position    
 */
void calibrateFilterMotor() {
  // spin motor until it reaches a physical block
  while(filterMotor.torque(torqueUnits::Nm) < 0.2) {
    filterMotor.spin(directionType::fwd, 5, velocityUnits::pct);
  }
  filterMotor.stop(brakeType::brake);
  filterMotor.resetPosition();
}


void drawDetectedBallsWithVisionSensors() {
  Brain.Screen.clearScreen();
  Brain.Screen.setOrigin(1, 1);
  Brain.Screen.drawRectangle(
      0, 0, 316,
      212); // pixel size of image:
            // https://help.vex.com/article/166-how-to-interpret-feedback-from-the-vision-sensor-and-vision-utility-interface
  Brain.Screen.drawLine(158, 0, 158, 212); // middle line (center)

  visionLeft.takeSnapshot(RED_BALL);
  if (visionLeft.largestObject.exists) {
    Brain.Screen.drawCircle(visionLeft.largestObject.centerX,
                            visionLeft.largestObject.centerY,
                            visionLeft.largestObject.width / 2);
    Brain.Screen.printAt(320, 20, false, "Red Ball Left Vision Sensor:");
    Brain.Screen.printAt(320, 40, false, "x-cntr: %d",
                         visionLeft.largestObject.centerX);
    Brain.Screen.printAt(320, 60, false, "width:  %d",
                         visionLeft.largestObject.width);
  }
  visionRight.takeSnapshot(RED_BALL);
  if (visionRight.largestObject.exists) {
    Brain.Screen.drawCircle(visionRight.largestObject.centerX,
                            visionRight.largestObject.centerY,
                            visionRight.largestObject.width / 2);
    Brain.Screen.printAt(320, 80, false, "Red Ball Right Vision Sensor:");
    Brain.Screen.printAt(320, 100, false, "x-cntr: %d",
                         visionRight.largestObject.centerX);
    Brain.Screen.printAt(320, 120, false, "width:  %d",
                         visionRight.largestObject.width);
  }
  if (visionLeft.largestObject.exists && visionRight.largestObject.exists) {
    Brain.Screen.drawCircle(
        (visionRight.largestObject.centerX + visionLeft.largestObject.centerX) /
            2,
        (visionRight.largestObject.centerY + visionLeft.largestObject.centerY) /
            2,
        (visionRight.largestObject.width + visionLeft.largestObject.width) / 2 /
            2,
        color::red);
    Brain.Screen.printAt(320, 260, false, "Combined Red Ball:");
    Brain.Screen.printAt(320, 280, false, "x-cntr: %d width: %d", (visionRight.largestObject.centerX + visionLeft.largestObject.centerX) / 2, (visionRight.largestObject.width + visionLeft.largestObject.width) / 2);
  }

  visionLeft.takeSnapshot(BLUE_BALL);
  if (visionLeft.largestObject.exists) {
    Brain.Screen.drawCircle(visionLeft.largestObject.centerX,
                            visionLeft.largestObject.centerY,
                            visionLeft.largestObject.width / 2);
    Brain.Screen.printAt(320, 140, false, "Blue Ball Left Vision Sensor:");
    Brain.Screen.printAt(320, 160, false, "x-cntr: %d",
                         visionLeft.largestObject.centerX);
    Brain.Screen.printAt(320, 180, false, "width:  %d",
                         visionLeft.largestObject.width);
  }
  visionRight.takeSnapshot(BLUE_BALL);
  if (visionRight.largestObject.exists) {
    Brain.Screen.drawCircle(visionRight.largestObject.centerX,
                            visionRight.largestObject.centerY,
                            visionRight.largestObject.width / 2);
    Brain.Screen.printAt(320, 200, false, "Blue Ball Right Vision Sensor:");
    Brain.Screen.printAt(320, 220, false, "x-cntr: %d",
                         visionRight.largestObject.centerX);
    Brain.Screen.printAt(320, 240, false, "width:  %d",
                         visionRight.largestObject.width);
  }
  if (visionLeft.largestObject.exists && visionRight.largestObject.exists) {
    Brain.Screen.drawCircle(
        (visionRight.largestObject.centerX + visionLeft.largestObject.centerX) /
            2,
        (visionRight.largestObject.centerY + visionLeft.largestObject.centerY) /
            2,
        (visionRight.largestObject.width + visionLeft.largestObject.width) / 2 /
            2,
        color::blue);
    Brain.Screen.printAt(320, 300, false, "Combined Blue Ball:");
    Brain.Screen.printAt(320, 320, false, "x-cntr: %d width: %d", (visionRight.largestObject.centerX + visionLeft.largestObject.centerX) / 2, (visionRight.largestObject.width + visionLeft.largestObject.width) / 2);

  }
}

/*
 * function runs infinitely (as part of autonomous program) to display vision sensor detections
 */
void drawDetectedBallsWithVisionSensorsInfinite() {
  drawDetectedBallsWithVisionSensors();
  wait(200, timeUnits::msec);
}

/*
function to get reliable sonar values:
  - measure (minimum) 5 times; wait 100ms between measurements
  - sort by value
  - eliminate 2 smallest and 2 highest values
  - return average of middle values
needs: <algorithm>, <vector>
see: http://www.cplusplus.com/reference/algorithm/sort/ & http://www.cplusplus.com/reference/algorithm/stable_sort/
*/
#include <algorithm>
#include <vector>
double getAverageSonicSensorValues(vex::sonar myDistanceSensor, int numberMeasurements = 7) {
  // check if minimum number of values is 5
  if (numberMeasurements < 5) {
    numberMeasurements = 5;
  }
  
  // declare array of #numberMeasurements double values
  double sensorValues[numberMeasurements]; 

  // for each array value retrieve on measurement from sensor
  for (int i = 0; i < sizeof(sensorValues); i++) {
    sensorValues[i] = myDistanceSensor.distance(distanceUnits::mm);
    // as the sonar sensor only provide updates after 100ms (field tests) we wait 100ms between measurements 
    wait(100, timeUnits::msec);
  }

  // sort array by values using vector
  // http://www.cplusplus.com/reference/algorithm/stable_sort/
  std::vector<double> myvector;
  myvector.assign(sensorValues,sensorValues + numberMeasurements);
  std::stable_sort(myvector.begin(), myvector.end());

  double myReturnValue = 0; // initialize return value
  // for each of the middle values: add to return value
  for (int i = 2; i < myvector.size() - 2; i++) {
    myReturnValue += myvector[i];
  }

  // return average of the middle values (without 2 smallest and 2 largest values)
  return myReturnValue / (sizeof(sensorValues) - 4.0);
}

/*
 * function checks with help of vision sensor looking at pre-load position what color is seen
 * first check on blue then on red
 */
void determineAllianceColor() {
  // check with blue ball signature on pre-load position (sensor visionFilter)
  visionFilter.takeSnapshot(BLUE_BALL);
  
  // if the signature returns an object and size of object is large enough means: blue ball detected -> redAlliance = false;
  if(visionFilter.largestObject.exists && visionFilter.largestObject.width > 100) {
    startInRedAlliance = false;
    printDebugMessageOnController("blue ball as pre-load detected");
    printDebugMessageOnBrain("blue ball as pre-load detected");
  }
  else {
    // check with blue ball signature on pre-load position (sensor visionFilter)
    visionFilter.takeSnapshot(RED_BALL);

    // if the signature returns an object and size of object is large enough means: red ball detected -> redAlliance = true;
    if(visionFilter.largestObject.exists && visionFilter.largestObject.width > 100) {
      startInRedAlliance = true;
      printDebugMessageOnController("red ball as pre-load detected");
      printDebugMessageOnBrain("red ball as pre-load detected");
    }
    else {
      printDebugMessageOnController("no ball as pre-load detected");
      printDebugMessageOnBrain("no ball as pre-load detected");
    }
  }
}

/*  drive sideways with different speeds and measure distance traveled using a
   sonic sensor this will allow to calibrate as different grounds may have a
   different grip and influence the precision values are read and written to a
   calibration file */
void measureDriveCalibrationValues(bool toBrainScreenOnly = false, int driveSpeed = 10, int durationMsec = 2000, int runs = 1) {
  std::string myfilename = getRandomStr(6) + "_driveCalibration.txt";
  bool fileReadyToWrite = openDebugFile(myfilename);
  Brain.Screen.print("start calibration with %d speed (%d runs)", driveSpeed, runs);
  Brain.Screen.newLine();
  if (!toBrainScreenOnly && fileReadyToWrite) {
    writeToDebugFile("start calibrating drive");
  }

  double startTime;
  double startDistance;
  for (int j = 1; j < runs + 1; j++) {
    while (!controllerPrimary.ButtonX.pressing()) {
      wait(50, timeUnits::msec);
    }
    Brain.resetTimer();
    startTime = Brain.timer(timeUnits::msec);
    startDistance = sonarRear.distance(distanceUnits::mm);

    if (!toBrainScreenOnly && fileReadyToWrite) {
      writeToDebugFile(toString(startTime) + ",start calibrating drive with " +
                       toString(driveSpeed) + "% motor speed");
      writeToDebugFile(toString(startTime) + ",run: " + toString(j));
      writeToDebugFile(toString(startTime) + ",start distance," +
                       toString(startDistance));
      writeToDebugFile("msec,distance,speed,measurement counter");
    }

    driveRobotForward(vex::directionType::fwd, driveSpeed);

    int i = 0;
    while (Brain.timer(timeUnits::msec) < durationMsec) {
      i++;
      if (!toBrainScreenOnly && fileReadyToWrite)
        writeToDebugFile(
            toString(Brain.timer(timeUnits::msec) - startTime) + "," +
            toString(sonarRear.distance(distanceUnits::mm) - startDistance) +
            ",Speed: " + toString(driveSpeed) + "%," + toString(i));
      wait(100, timeUnits::msec);
    }
    stopDriveTrain();
    double endDistance = sonarRear.distance(distanceUnits::mm);
    double endTime = Brain.timer(timeUnits::msec);
    if (!toBrainScreenOnly && fileReadyToWrite) {
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",Stop measurements for calibration");
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",end distance," + toString(endDistance));
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",distance traveled," +
                       toString(endDistance - startDistance));
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",time needed (ms)," + toString(endTime - startTime));
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",average speed for " + toString(driveSpeed) +
                       "% motor speed (mm/msec)," +
                       toString((endDistance - startDistance) /
                                (endTime - startTime) * 1000) +
                       "mm/s");
    }
    if (toBrainScreenOnly) {
      printDebugMessageOnBrain("start: " + toString(startDistance)+ "mm, end: " + toString(endDistance));
      printDebugMessageOnBrain("distance diff: " + toString(endDistance-startDistance) + " mm");
      printDebugMessageOnBrain("time needed: " + toString(endTime-startTime) + " msec");
    }
    printDebugMessageOnBrain("average speed for " + toString(driveSpeed) + " motor-speed: " + toString((endDistance - startDistance) / (endTime - startTime) * 1000) + " mm/s");
  }
  closeDebugFile();
}

/* spin robot with different speeds and measure degrees the robot hat turned
   using the gyro sensor; this will allow to calibrate as different grounds may
   have a different grip level and influence the precision;
   values are read and written to a calibration file */
void measureTurnCalibrationValues(bool toBrainScreenOnly = false, int turnSpeed = 10, int durationMsec = 2000, int runs = 1) {
  std::string myfilename = getRandomStr(6) + "_turnCalibration.txt";
  bool fileReadyToWrite = openDebugFile(myfilename);
  if (!toBrainScreenOnly && fileReadyToWrite) {
    writeToDebugFile("start calibrating");
  }

  double startTime;
  double startAngle;
  for (int j = 1; j < runs + 1; j++) {
    while (!controllerPrimary.ButtonX.pressing()) {
      wait(50, timeUnits::msec);
    }
    Brain.resetTimer();
    gyroSensor.resetRotation();
    startTime = Brain.timer(timeUnits::msec);
    startAngle = gyroSensor.rotation(rotationUnits::deg);

    if (!toBrainScreenOnly && fileReadyToWrite) {
      writeToDebugFile(toString(startTime) + ",start calibrating with " +
                       toString(turnSpeed) + "% motor speed");
      writeToDebugFile(toString(startTime) + ",run: " + toString(j));
      writeToDebugFile(toString(startTime) + ",start angle," +
                       toString(startAngle));
      writeToDebugFile("msec,distance,speed,measurement counter");
    }

    turnRobot(vex::directionType::fwd, turnSpeed);

    int i = 0;
    while (Brain.timer(timeUnits::msec) < durationMsec) {
      i++;
      if (!toBrainScreenOnly && fileReadyToWrite)
        writeToDebugFile(
            toString(Brain.timer(timeUnits::msec) - startTime) + "," +
            toString(gyroSensor.rotation(rotationUnits::deg) - startAngle) +
            ",turn speed: " + toString(turnSpeed) + "%," + toString(i));
      wait(10, timeUnits::msec);
    }
    stopDriveTrain();
    double endAngle = gyroSensor.rotation(rotationUnits::deg);
    double endTime = Brain.timer(timeUnits::msec);
    if (!toBrainScreenOnly && fileReadyToWrite) {
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",Stop measurements for calibration");
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",end distance," + toString(endAngle));
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",distance traveled," +
                       toString(endAngle - startAngle));
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",time needed (ms)," + toString(endTime - startTime));
      writeToDebugFile(toString(Brain.timer(timeUnits::msec)) +
                       ",average speed for " + toString(turnSpeed) +
                       "% motor speed (mm/msec)," +
                       toString((endAngle - startAngle) /
                                (endTime - startTime) * 1000) +
                       "mm/s");
    }
    printDebugMessageOnBrain("average turn speed for " + toString(turnSpeed) + " motor speed: " + toString((endAngle - startAngle) / (endTime - startTime) * 1000) + " deg/s");
  }
  closeDebugFile();

}

//transport ball trugh robot
void transportBallThroughRobot(int spinTime,int motorVelocity) {
  outputMotor.spinFor(directionType::fwd, spinTime, timeUnits::sec, motorVelocity, velocityUnits::pct);
  transportMotor.spinFor(directionType::fwd, spinTime, timeUnits::sec, motorVelocity, velocityUnits::pct);
  intakeMotor.spinFor(directionType::fwd, spinTime, timeUnits::sec, motorVelocity, velocityUnits::pct);
}

// sort out wrong ball (by color) 
void checkBallInFilter() {
  if (limitSwitchFilterPosition.pressing()) {
    if (startInRedAlliance) { // sort out blue ball
      visionFilter.takeSnapshot(BLUE_BALL_DARK);
    }
    else { // sort out red ball
      visionFilter.takeSnapshot(RED_BALL_DARK);
    }
    
    // ball of oponent color is in filter position spill it our (using function filterMoveOpen())
    // we use a minimum size to not react on something in the background
    if(visionFilter.largestObject.exists && visionFilter.largestObject.width > 100) {
      filterMoveOpen();
      wait(1, timeUnits::sec);
      filterMoveClosed();
    }
  }
}

// toDo: implement logic (when ball releases filterLimitSwitch)
void ballInFilterReleased() {
  
}