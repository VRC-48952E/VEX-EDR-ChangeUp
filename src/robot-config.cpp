#include "vex.h"
#include "helperfunctions.h"

//include from VisionSensorConfig Tool
#include "VisionSensorConfig.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// global variable if in red or blue alliance
// determined during init function with vision sensor on color of pre-load: determineAllianceColor()
bool startInRedAlliance = true;

// variable to define if balls of other alliance should be filtered automatically
bool ballAutoFilter = false;

// define your global instances of motors and other devices here
vex::motor leftFrontMotor = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1,false);
vex::motor rightFrontMotor = vex::motor(vex::PORT13, vex::gearSetting::ratio18_1,false);
vex::motor leftRearMotor = vex::motor(vex::PORT16, vex::gearSetting::ratio18_1,false);
vex::motor rightRearMotor = vex::motor(vex::PORT12, vex::gearSetting::ratio18_1,false);

vex::motor intakeMotor = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1,true);
vex::motor filterMotor = vex::motor(vex::PORT15, vex::gearSetting::ratio18_1,false);
vex::motor transportMotor = vex::motor(vex::PORT8, vex::gearSetting::ratio6_1,true);
vex::motor outputMotor = vex::motor(vex::PORT6, vex::gearSetting::ratio6_1,true);

vex::sonar sonarRight = vex::sonar(Brain.ThreeWirePort.C); //C is the output of sonar, D is the input
vex::sonar sonarLeft = vex::sonar(Brain.ThreeWirePort.E); //E is the output of sonar, F is the input
vex::sonar sonarRear = vex::sonar(Brain.ThreeWirePort.G); //G is the output of sonar, H is the input
vex::distance distanceFront = vex::distance(vex::PORT7);


vex::limit limitSwitchFilterPosition = vex::limit(Brain.ThreeWirePort.B);
vex::limit limitSwitchShootingPosition = vex::limit(Brain.ThreeWirePort.A);

vex::inertial gyroSensor = vex::inertial(vex::PORT4);
vex::controller controllerPrimary = vex::controller(controllerType::primary);
vex::controller controllerPartner = vex::controller(controllerType::partner);


// VEXcode device constructors -> use signatures from VisionSensorConfig
// these vision sensors are for aligning on balls and towers
vision visionLeft  = vision (PORT2, 50, BLUE_BALL, RED_BALL, BLUE_BALL_DARK, RED_BALL_DARK);
vision visionRight = vision (PORT3, 50, BLUE_BALL, RED_BALL, BLUE_BALL_DARK, RED_BALL_DARK);

//this vision sensor is for the filter motor to know whitch ball is on the filter
vision visionFilter = vision (PORT10, 50, BLUE_BALL_DARK, RED_BALL_DARK); 

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  controllerPrimary.Screen.setCursor(1, 1);
  controllerPrimary.Screen.print("Calibrating Gyro for Drivetrain...");
  controllerPrimary.Screen.newLine();
  gyroSensor.calibrate();
  wait(200, msec);
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (gyroSensor.isCalibrating()) {
    wait(25, msec);
  }
  controllerPrimary.Screen.clearLine();
  controllerPrimary.Screen.print("Gyro initialized."); 

  calibrateFilterMotor();

  //printDebugMsgOnController("Initialize Cube Holder");
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);

  // check on ball color as pre-load
  determineAllianceColor();

  // register handling functions if ball is in filter position or limit switch is released
  limitSwitchFilterPosition.pressed(checkBallInFilter);
  limitSwitchFilterPosition.released(ballInFilterReleased);
}