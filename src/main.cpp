/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Julius Rohr                                               */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "autonomous.h"
#include "helperfunctions.h"
#include "usercontrol.h"

using namespace vex;

// A global instance of vex::competition
vex::competition Competition;

int main() {
  // Initializing Robot Configuration in the beginning
  vexcodeInit();

  /*measureDriveCalibrationValues(false, 10, 3500, 4);
  measureDriveCalibrationValues(false, 20, 3500, 4);
  measureDriveCalibrationValues(false, 60, 2000, 4);
  measureTurnCalibrationValues(false, 10, 3000, 4);
  measureTurnCalibrationValues(false, 20, 3000, 4);
  measureTurnCalibrationValues(false, 30, 3000, 4);
  measureTurnCalibrationValues(false, 40, 3000, 4);*/

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  //int jumper = Brain.ThreeWirePort.E.value();

  //Brain.ThreeWirePort.A.set(true);
  //printDebugMessageOnBrain("LED: ON");
  
  // Prevent main from exiting with an infinite loop.
  while (1) {
    //drawDeteedCubesWithVisionSensors();
    Brain.Screen.clearScreen();
    drawDetectedBallsWithVisionSensors();

    
    Brain.Screen.printAt(0,120, false, "GyroHeading: %f",gyroSensor.heading());
    Brain.Screen.printAt(0,140, false, "Front Distance: %f",distanceFront.objectDistance(distanceUnits::mm));
    Brain.Screen.printAt(0,160, false, "Rear  Distance: %f",sonarRear.distance(distanceUnits::mm));
    Brain.Screen.printAt(0,180, false, "Right Distance: %f",sonarRight.distance(distanceUnits::mm));
    Brain.Screen.printAt(0,200, false, "Left  Distance: %f",sonarLeft.distance(distanceUnits::mm));

    //Brain.Screen.printAt(0,100, false, "ArmMotor Temp: %f",LeftRearMotor.temperature(temperatureUnits::celsius));
  /*    controllerPrimary.Screen.setCursor(1, 1);
    controllerPrimary.Screen.print("Left Front Motor Tmp: %f",leftFrontMotor.temperature(temperatureUnits::celsius));
    controllerPrimary.Screen.setCursor(2, 1);
    controllerPrimary.Screen.print("Right Front Motor Tmp: %f",rightFrontMotor.temperature(temperatureUnits::celsius));
    controllerPrimary.Screen.setCursor(3, 1);
    controllerPrimary.Screen.print("Axis: %f",controllerPrimary.Axis2.value());
    */

    vex::task::sleep(50); // Sleep the task for a short amount of time to
  }
}
