#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// This file instantiates all of the motos, sensors, and global variables we use for our robot.
brain Brain;

controller Controller1 = controller();

motor frontRightMotor = motor(PORT7, ratio18_1, false); 
motor frontLeftMotor = motor(PORT20, ratio18_1, true);
motor middleRightMotor = motor(PORT5, ratio18_1, true);
motor middleLeftMotor = motor(PORT15, ratio18_1, false);
motor backRightMotor = motor(PORT3, ratio18_1, false); 
motor backLeftMotor = motor(PORT11, ratio18_1, true); 
motor intake = motor(PORT1, ratio18_1, true);
motor lift1 = motor(PORT6, ratio18_1, true);
inertial gyroInertialSensor = inertial(PORT21);
encoder trackingLeft = encoder(Brain.ThreeWirePort.C); 
encoder trackingRight = encoder(Brain.ThreeWirePort.G); 
encoder back = encoder(Brain.ThreeWirePort.F);        
pneumatics piston = pneumatics(Brain.ThreeWirePort.E);
pneumatics backPiston = pneumatics(Brain.ThreeWirePort.A);

float globalX = 0;
float globalY = 0;
float globalAngle = 0;

void vexcodeInit(void) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  gyroInertialSensor = inertial(PORT21);
  wait(200, msec);
  gyroInertialSensor.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (gyroInertialSensor.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  wait(2000, msec);
  Brain.Screen.clearScreen();
  gyroInertialSensor.setRotation(0, rotationUnits::deg);
}
