using namespace vex;
using signature = vex::vision::signature;

extern controller controllerPrimary;

// robot motion 
void stopDriveTrain(brakeType myBrakeType = brakeType::brake);
void turnRobot(directionType spinDirection, int spinVelocity);
void turnRobotToAngle(int targetAngle, int turnSpeed = 20, int tolerance = 3);
void turnRobotByAngle(int angleToRotate);
void driveRobotForward(directionType spinDirection, int spinVelocity);
void driveRobotSideways(directionType spinDirection, int spinVelocity);
void driveRobotFor(int distanceToTravel, directionType mydirection = directionType::fwd, int driveSpeed = 20);  
void driveFrontUntilDistance(int targetDistance, int tolerance = 5, distanceUnits mydistanceUnit = distanceUnits::cm, int driveSpeedMax = 40);
void driveRearUntilDistance(int targetDistance, int tolerance = 5, distanceUnits mydistanceUnit = distanceUnits::cm);
void driveRightUntilDistance(int targetDistance, int tolerance = 5, distanceUnits mydistanceUnit = distanceUnits::cm);
void driveLeftUntilDistance(int targetDistance, int tolerance = 5, distanceUnits mydistanceUnit = distanceUnits::cm);
void filterMoveOpen();
void filterMoveClosed();
void transportBallThroughRobot(int spinTime,int motorVelocity);
void intakeBallUntilFilterPosition();
void getBallInShootingPosition(bool controlTransport = true, bool controlIntake = false);
void getBallInShootingPosition();
void checkBallInFilter();
void ballInFilterReleased();

// autonomous operations helper functions
void determineAllianceColor();
double getAverageSonicSensorValues(vex::sonar myDistanceSensor, int numberMeasurements = 7);
void centerOnBall(signature vision_signature, std::string alignmentMethod = "sideways", int tolerance = 10, int alignmentSpeed = 10);
void approachBallUntilWidthReached(signature vision_signature, int widthToAlign, int tolerance = 10, int alignmentSpeed = 10);
void followBall(signature vision_signature);

// start sequence
void calibrateFilterMotor(); // ensure proper filter motor position
void releaseUpperRail(); // release upper rail for ball transport 
void releaseSideArms();  // release side arms that are sticked to both robot side

// visualize functions 
void drawDetectedBallsWithVisionSensors();
void drawDetectedBallsWithVisionSensorsInfinite();

// debugging functions
template <typename T> std::string toString(T value);
bool openDebugFile(std::string fileName = "BestGhosts_Debug_File.txt");
void closeDebugFile();
void writeToDebugFile(std::string debugMessage); 
void printDebugMessageOnBrain(std::string debugMessage);
void printDebugMessageOnController(std::string debugMessage, vex::controller controllerToPrintTo = controllerPrimary);

// calibration functions
void measureDriveCalibrationValues(bool toBrainScreenOnly = false, int driveSpeed = 10, int durationMsec = 2000, int runs = 1);
void measureTurnCalibrationValues(bool toBrainScreenOnly = false, int turnSpeed = 10, int durationMsec = 2000, int runs = 1);

