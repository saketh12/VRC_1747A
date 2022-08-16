#include "vex.h"
using namespace vex;

#include "myOpControl.h"
#include "robot-config.h"
#include "utils.h"


#include <fstream>
#include <iomanip>
#include <iostream>

int deadband = 4;

int temperaturecounter = 0;
int first = 1;
bool inbrakeMode = false;
// Contains all of our driver control functions that are run simultaneously using c++ threads.

void moveLift2DownOp() {

  intake.setRotation(0, rotationUnits::deg);
  intake.setVelocity(100, percentUnits::pct);
  while (intake.rotation(rotationUnits::deg) < 50) {
    intake.spin(directionType::fwd);
  }
  backPiston.open();
  wait(200, msec);
  intake.setVelocity(-100, percentUnits::pct);
  while (intake.rotation(rotationUnits::deg) > -750) {
    intake.spin(directionType::fwd);
  }
  intake.stop(brakeType::coast);
}

void moveLift2UpOp() {

  intake.setRotation(0, rotationUnits::deg);
  intake.setVelocity(100, percentUnits::pct);
  while (intake.rotation(rotationUnits::deg) < 375) {
    intake.spin(directionType::fwd);
  }
  backPiston.close();
  intake.stop(brakeType::coast);
}

void handlePiston() {
  if ((float)(piston.value()) == 1) {
    piston.close();
  } else {
    piston.open();
  }
}

void handleBackPiston() {
  if ((float)(backPiston.value()) == 1) {
    backPiston.close();
  } else {
    backPiston.open();
  }
}

void handleLift1() {
  if (Controller1.ButtonR2.pressing()) {
    lift1.setVelocity(100, percent);
  }

  else if (Controller1.ButtonR1.pressing()) {
    lift1.setVelocity(-100, percent);
  }

  else {
    lift1.setVelocity(0, percent);
    lift1.setBrake(brakeType::brake);
  }
  lift1.spin(directionType::fwd);
}

void handleIntake() {
  if (Controller1.ButtonL2.pressing()) {
    intake.setVelocity(100, percent);
  }

  else if (Controller1.ButtonL1.pressing()) {
    intake.setVelocity(-100, percent);
  }

  else {
    intake.setVelocity(0, percent);
    intake.setBrake(brakeType::brake);
  }
  intake.spin(directionType::fwd);
}

void switchToBrakeMode() {
  if (inbrakeMode) {
    inbrakeMode = false;
  } else {
    inbrakeMode = true;
  }
}

void handleRobotDrive() {
  first = 1;

  int leftFrontMotorSpeed, leftBackMotorSpeed, rightFrontMotorSpeed,
      rightBackMotorSpeed;
  handleBackPiston();
  handlePiston();
  while (true) {

    leftFrontMotorSpeed =
        Controller1.Axis1.position() * 0.5 + Controller1.Axis3.position();

    leftBackMotorSpeed =
        Controller1.Axis1.position() * 0.5 + Controller1.Axis3.position();

    rightFrontMotorSpeed =
        Controller1.Axis1.position() * 0.5 - Controller1.Axis3.position();

    rightBackMotorSpeed =
        Controller1.Axis1.position() * 0.5 - Controller1.Axis3.position();
    // Set the speed of the left motor. If the value is less than the deadband,
    // set it to zero.
    if (abs(leftBackMotorSpeed) < deadband) {
      // Set the speed to zero.
      frontLeftMotor.setVelocity(0, percent);
      middleLeftMotor.setVelocity(0, percent);
      backLeftMotor.setVelocity(0, percent);

    } else {
      // Set the speed to leftMotorSpeed
      frontLeftMotor.setVelocity(leftFrontMotorSpeed, percent);
      middleLeftMotor.setVelocity(leftFrontMotorSpeed, percent);
      backLeftMotor.setVelocity(leftBackMotorSpeed, percent);
    }

    // Set the speed of the right motor. If the value is less than the deadband,
    // set it to zero.
    if (abs(rightFrontMotorSpeed) < deadband &&
        abs(rightBackMotorSpeed) < deadband) {
      frontRightMotor.setVelocity(0, percent);
      middleRightMotor.setVelocity(0, percent);
      backRightMotor.setVelocity(0, percent);

    } else {
      // Set the speed to rightMotorSpeed
      frontRightMotor.setVelocity(rightFrontMotorSpeed, percent);
      middleRightMotor.setVelocity(rightFrontMotorSpeed, percent);
      backRightMotor.setVelocity(rightBackMotorSpeed, percent);
    }

    // Spin both motors in the forward direction.
    frontLeftMotor.spin(forward);
    middleLeftMotor.spin(forward);
    middleRightMotor.spin(reverse);
    frontRightMotor.spin(reverse);
    backLeftMotor.spin(forward);
    backRightMotor.spin(reverse);

    if (leftFrontMotorSpeed == 0 && leftBackMotorSpeed == 0 &&
        rightBackMotorSpeed == 0 && rightFrontMotorSpeed == 0) {
      if (!inbrakeMode) {
        frontLeftMotor.setBrake(brakeType::coast);
        middleLeftMotor.setBrake(brakeType::coast);
        frontRightMotor.setBrake(brakeType::coast);
        middleRightMotor.setBrake(brakeType::coast);
        backLeftMotor.setBrake(brakeType::coast);
        backRightMotor.setBrake(brakeType::coast);
      } else {
        frontLeftMotor.setBrake(brakeType::brake);
        middleLeftMotor.setBrake(brakeType::brake);
        frontRightMotor.setBrake(brakeType::brake);
        middleRightMotor.setBrake(brakeType::brake);
        backLeftMotor.setBrake(brakeType::brake);
        backRightMotor.setBrake(brakeType::brake);
      }
    }
    Controller1.ButtonA.pressed(switchToBrakeMode);
    Controller1.ButtonX.pressed(handlePiston);
    Controller1.ButtonB.pressed(handleBackPiston);
    Controller1.ButtonUp.pressed(moveLift2UpOp);
    Controller1.ButtonDown.pressed(moveLift2DownOp);
    handleLift1();
    handleIntake();
    wait(10, msec);
  }
}
