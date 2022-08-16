#include "pidGyroTurn.h"

#include "vex.h"
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace vex;
// Contains all the code for the PID controller we developed to make our robot do accurate turns
typedef struct {

  vex::inertial gyro = gyroInertialSensor;
  double target;
  double kP;
  double kI;
  double kD;
  double error;
  double previous_error;
  double integral;
  double integralActiveZone;
  double integralPowerLimt;
  double derivative;

} PIDController;

void stopMotorsTurn(bool completeStop) {

  // Left Motors Break Type
  frontLeftMotor.setVelocity(0, percentUnits::pct);
  middleLeftMotor.setVelocity(0, percentUnits::pct);
  backLeftMotor.setVelocity(0, percentUnits::pct);

  frontRightMotor.setVelocity(0, percentUnits::pct);
  middleRightMotor.setVelocity(0, percentUnits::pct);
  backRightMotor.setVelocity(0, percentUnits::pct);

  frontLeftMotor.stop(brakeType::brake);
  middleLeftMotor.stop(brakeType::brake);
  backLeftMotor.stop(brakeType::brake);

  // Right Motors Break Type
  frontRightMotor.stop(brakeType::brake);
  middleRightMotor.stop(brakeType::brake);
  backRightMotor.stop(brakeType::brake);
}
void setPowerTurn(int leftPower, int rightPower) {
  // Left Motor Power
  frontLeftMotor.setVelocity(leftPower, percentUnits::pct);
  middleLeftMotor.setVelocity(leftPower, percentUnits::pct);
  backLeftMotor.setVelocity(leftPower, percentUnits::pct);

  // Right Motor Power
  frontRightMotor.setVelocity(-1 * rightPower, percentUnits::pct);
  middleRightMotor.setVelocity(-1 * rightPower, percentUnits::pct);
  backRightMotor.setVelocity(-1 * rightPower, percentUnits::pct);
}

void setPowerDoubleTurn(double leftPower, double rightPower) {
  // Left Motor Power
  frontLeftMotor.setVelocity(leftPower, percentUnits::pct);
  middleLeftMotor.setVelocity(leftPower, percentUnits::pct);
  backLeftMotor.setVelocity(leftPower, percentUnits::pct);

  // Right Motor Power
  frontRightMotor.setVelocity(rightPower, percentUnits::pct);
  middleRightMotor.setVelocity(rightPower, percentUnits::pct);
  backRightMotor.setVelocity(rightPower, percentUnits::pct);
}

void spinMotorsTurn() {
  // Left Motors
  frontLeftMotor.spin(directionType::fwd);
  middleLeftMotor.spin(directionType::fwd);
  backLeftMotor.spin(directionType::fwd);

  // Right Motors
  frontRightMotor.spin(directionType::fwd);
  middleRightMotor.spin(directionType::fwd);
  backRightMotor.spin(directionType::fwd);
}

double getGyroValue(inertial sensor, std::ofstream &fs) {
  double gyroVal = sensor.heading(vex::degrees);
  fs << "actual Gyro Value: " << gyroVal << "\r\n";
  if (gyroVal >= 358.0 || gyroVal <= 2.0) {
    gyroVal = 0.0;
  }
  return gyroVal;
}
//<<<<<<<<<<<<<<<<<<--------------0---------------->>>>>>>>>>>>>>>>>>>
void initalizePIDControllerTurn(PIDController &controller,
                                inertial target_sensor, double kP, double kI,
                                double kD, double integralActiveZone = 20,
                                double integralPowerLimt = 15) {
  controller.gyro = target_sensor;
  controller.kP = kP;
  controller.kI = kI;
  controller.kD = kD;
  controller.integral = 0;
  controller.integralActiveZone =
      integralActiveZone; // ---- are integralActiveZone
  controller.integralPowerLimt = integralPowerLimt;
  controller.error = 0;
}

double updatePIDControllerTurn(PIDController &controller, double target,
                               double maxPowerCoefficent, int delay = 10) {

  // Save the previous error for the derivative
  controller.previous_error = controller.error;

  // Calculate a new error by finding the difference between the current
  // position and the desired position.
  double gyroVal = (double)gyroInertialSensor.rotation();
  controller.error = target - gyroVal;

  Controller1.Screen.setCursor(4, 0);
  Controller1.Screen.print("Error: %5.2f", controller.error);

  // Begin summing the errors into the integral term if the error is below a
  // threshold, and reset it if not. This is to prevent the integral from
  // growing too large.
  double absError =
      (controller.error < 0) ? -1 * controller.error : controller.error;
  if (absError <= 1.5) {
    printf("Angle %f \n", (float)gyroInertialSensor.rotation());
    return 0;
  }

  if (absError < controller.integralActiveZone && absError != 0) {
    controller.integral += controller.error;
  } else {
    controller.integral = 0;
  }

  if (controller.kI != 0) {
    double maxIntegralLimit = controller.integralPowerLimt / controller.kI;
    if (controller.integral > maxIntegralLimit) {
      controller.integral = maxIntegralLimit;
    } else if (controller.integral < -controller.integralPowerLimt) {
      controller.integral = -(controller.integralPowerLimt);
    }
  }

  // Calculate the derivative by finding the change between the current error
  // and last update's error
  controller.derivative = controller.error - controller.previous_error;

  // Delay the function from returning. This is useful when it this is the last
  // PIDControllerUpdate in a task, and we don't need to calculate over small
  // time periods.
  wait(delay, msec);

  double powerFromProportion = controller.kP * controller.error;
  double powerFromIntegral = controller.kI * controller.integral;
  double powerFromDerivative = controller.kD * controller.derivative;
  // Combine all the parts of the PID function into the PID algorithm and return
  // the value.
  double power = powerFromProportion + powerFromIntegral + powerFromDerivative;

  if (power > maxPowerCoefficent * 100) {
    power = maxPowerCoefficent * 100;
  } else if (power < -maxPowerCoefficent * 100) {
    power = -maxPowerCoefficent * 100;
  }
  printf("ER %f power %f powerFromProportion %f powerFromIntegral %f "
         "powerFromDerivative %f GV %f Timer %f\n",
         controller.error, power, powerFromProportion, powerFromIntegral,
         powerFromDerivative, gyroVal, Brain.timer(timeUnits::sec));
  Brain.Screen.setCursor(5, 0);
  Brain.Screen.print("Power: %5.2f", power);

  return power;
}

void invokePIDTurn(double target, double kP, double kI, double kD,
                   double maxPowerCoefficent) {
  PIDController turnController;
  Brain.resetTimer();
  initalizePIDControllerTurn(turnController, gyroInertialSensor, kP, kI,
                             kD); // 0.5, 0.07, 0.0  // 0.33, 0.07, 0.0

  double curTurnError = 0;
  while (true) {
    double timeElapsed = Brain.timer(timeUnits::sec);
    double power =
        updatePIDControllerTurn(turnController, target, maxPowerCoefficent);
    double error = abs(target - (double)gyroInertialSensor.rotation());

    if (error <= 1.0) {
      printf("Exited because of accuracy %f", gyroInertialSensor.rotation());
      stopMotorsTurn(true);
      break;
    }
    if (abs(power) < 5) {
      if (power < 0) {
        power = -5;
      } else {
        power = 5;
      }
    }
    setPowerDoubleTurn(power, -power);
    Brain.Screen.printAt(20, 100, "time: %f", Brain.timer(timeUnits::sec));

    spinMotorsTurn();

    Controller1.Screen.setCursor(1, 0);
    Controller1.Screen.print("Gyro: %5.2f", gyroInertialSensor.heading());
    wait(100, msec);
  }
  frontLeftMotor.setBrake(brakeType::brake);
  middleLeftMotor.setBrake(brakeType::brake);
  backLeftMotor.setBrake(brakeType::brake);

  // Right Motors
  frontRightMotor.setBrake(brakeType::brake);
  middleRightMotor.setBrake(brakeType::brake);
  backRightMotor.setBrake(brakeType::brake);
}