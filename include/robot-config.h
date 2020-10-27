using namespace vex;
using signature = vision::signature;

// global settings
extern bool startInRedAlliance;
extern bool ballAutoFilter;

// VEXcode devices
extern brain Brain;
extern motor leftFrontMotor;
extern motor rightFrontMotor;
extern motor leftRearMotor;
extern motor rightRearMotor;
extern sonar sonarLeft; 
extern sonar sonarRight; 
extern sonar sonarRear;
extern distance distanceFront;
extern limit limitSwitchFilterPosition;
extern limit limitSwitchShootingPosition;
extern inertial gyroSensor;
extern controller controllerPrimary;
extern controller controllerPartner;
extern motor intakeMotor;
extern motor filterMotor;
extern motor transportMotor;
extern motor outputMotor;

extern signature BLUE_BALL;
extern signature BLUE_BALL_DARK;
extern signature RED_BALL;
extern signature RED_BALL_DARK;

extern vision visionLeft;
extern vision visionRight;
extern vision visionFilter;

void  vexcodeInit( void );