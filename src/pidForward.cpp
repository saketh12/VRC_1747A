#include "pidForward.h"

#include "vex.h"
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace vex;

double startEncoderValue;

// Contains all the code for the PID controller we developed to make our robot
// perform accurate linear motion
typedef struct {

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
  const double circumference = 8.635;
  const double ticksPerRevolution = 360.0;
} PIDController;

double calcDifferenceInAngles(double current = 3, double target = 0) {
  if (target > current) {
    if (((current + 360) - target) < (target - current)) {
      return (current + 360) - target;
    } else {
      return target - current;
    }
  } else {
    if (((target + 360) - current) < (current - target)) {
      return (target + 360) - current;
    } else {
      return current - target;
    }
  }
}

bool correctionYes = false;

void stopMotorsDrive(bool completeStop) {
  if (completeStop) {
    // Left Motors Break Type
    frontLeftMotor.stop(brakeType::brake);
    middleLeftMotor.stop(brakeType::brake);
    backLeftMotor.stop(brakeType::brake);

    // Right Motors Break Type
    frontRightMotor.stop(brakeType::brake);
    middleRightMotor.stop(brakeType::brake);
    backRightMotor.stop(brakeType::brake);
  } else {
    // Left Motors
    frontLeftMotor.stop();
    middleLeftMotor.stop();
    backLeftMotor.stop();

    // Right Motos
    frontRightMotor.stop();
    middleRightMotor.stop();
    backRightMotor.stop();
  }
}
void setPowerDrive(int leftPower, int rightPower) {
  // Left Motor Power
  frontLeftMotor.setVelocity(leftPower, percentUnits::pct);
  middleLeftMotor.setVelocity(leftPower, percentUnits::pct);
  backLeftMotor.setVelocity(leftPower, percentUnits::pct);

  // Right Motor Power
  frontRightMotor.setVelocity(rightPower, percentUnits::pct);
  middleRightMotor.setVelocity(rightPower, percentUnits::pct);
  backRightMotor.setVelocity(rightPower, percentUnits::pct);
}

void setPowerDoubleDrive(double leftPower, double rightPower,
                         double angleOfDirection = 0) {

  // Left Motor Power
  float leftPos = trackingRight.position(rotationUnits::deg);
  float rightPos = leftPos;
  // printf("before rightPowr:%f leftPower:%f  leftPos:%f
  // rightPos:%f\n",rightPower,leftPower,leftPos,rightPos);

  double current = gyroInertialSensor.heading(degrees);

  bool increasingRight;

  // Determines which Direction you need to turn
  if (angleOfDirection > current) {
    if (((current + 360) - angleOfDirection) < (angleOfDirection - current)) {
      increasingRight = true;
    } else {
      increasingRight = false;
    }
  } else {
    if (((angleOfDirection + 360) - current) < (current - angleOfDirection)) {
      increasingRight = false;
    } else {
      increasingRight = true;
    }
  }
  double correction = calcDifferenceInAngles(current, angleOfDirection);
  if (correction > 1) {
    if (increasingRight) {
      rightPower = rightPower + correction * 1.2;
      printf("RightPower increase current %f, angleOfDirection %f correction "
             "%f rightPower %f leftPower %f",
             current, angleOfDirection, correction, rightPower, leftPower);
    } else {
      leftPower = leftPower + correction * 1.2;
      printf("LeftPower increase current %f, angleOfDirection %f correction %f "
             "rightPower %f  leftPower %f",
             current, angleOfDirection, correction, rightPower, leftPower);
    }
  } else {
    printf("no increase current %f, angleOfDirection %f correction %f "
           "rightPower %f  leftPower %f",
           current, angleOfDirection, correction, rightPower, leftPower);
  }
  frontLeftMotor.setVelocity(leftPower, percentUnits::pct);
  middleLeftMotor.setVelocity(leftPower, percentUnits::pct);
  backLeftMotor.setVelocity(leftPower, percentUnits::pct);

  // Right Motor Power
  frontRightMotor.setVelocity(rightPower, percentUnits::pct);
  middleRightMotor.setVelocity(rightPower, percentUnits::pct);
  backRightMotor.setVelocity(rightPower, percentUnits::pct);
}
void setBackwardPowerDoubleDrive(double leftPower, double rightPower,
                                 double angleOfDirection = 0) {

  double current = gyroInertialSensor.heading(degrees);

  bool increasingRight;

  // Determines which Direction you need to turn
  if (angleOfDirection > current) {
    if (((current + 360) - angleOfDirection) < (angleOfDirection - current)) {
      increasingRight = false;
    } else {
      increasingRight = true;
    }
  } else {
    if (((angleOfDirection + 360) - current) < (current - angleOfDirection)) {
      increasingRight = true;
    } else {
      increasingRight = false;
    }
  }

  // Left Motor Power
  frontLeftMotor.setVelocity(-leftPower, percentUnits::pct);
  middleLeftMotor.setVelocity(-leftPower, percentUnits::pct);
  backLeftMotor.setVelocity(-leftPower, percentUnits::pct);

  // Right Motor Power
  frontRightMotor.setVelocity(-rightPower, percentUnits::pct);
  middleRightMotor.setVelocity(-rightPower, percentUnits::pct);
  backRightMotor.setVelocity(-rightPower, percentUnits::pct);
}
void spinMotorsDrive() {
  // Left Motors
  frontLeftMotor.spin(directionType::fwd);
  middleLeftMotor.spin(directionType::fwd);
  backLeftMotor.spin(directionType::fwd);

  // Right Motors
  frontRightMotor.spin(directionType::fwd);
  middleRightMotor.spin(directionType::fwd);
  backRightMotor.spin(directionType::fwd);
}

double getEncoderValue(encoder sensor, std::ofstream &fs) {
  double encoderVal = sensor.position(rotationUnits::deg);
  printf("actual Encoder Value: %f \n", encoderVal);
  return encoderVal;
}
double convertToTicks(PIDController &controller, double distance) {
  double revolutions = distance / controller.circumference;
  double totalTicks = revolutions * controller.ticksPerRevolution;
  return totalTicks;
}
double convertToInches(PIDController &controller, double ticks) {
  double revolutions = ticks / controller.ticksPerRevolution;
  double totalInches = revolutions * controller.circumference;
  return totalInches;
}
//<<<<<<<<<<<<<<<<<<--------------0---------------->>>>>>>>>>>>>>>>>>>
void initalizePIDControllerDrive(PIDController &controller,
                                 encoder target_sensor, double kP, double kI,
                                 double kD, double integralActiveZone = 30,
                                 double integralPowerLimt = 15) {

  controller.kP = kP;
  controller.kI = kI;
  controller.kD = kD;
  controller.integral = 0;
  controller.integralActiveZone =
      integralActiveZone; // ---- are integralActiveZone
  controller.integralPowerLimt = integralPowerLimt;
  controller.error = 0;
  // target_sensor.resetPosition();
}

double updatePIDControllerDrive(PIDController &controller, double target,
                                bool useY, std::ofstream &fs,
                                double angleOfDirection = 0,
                                double maxPowerCoefficent = 0.60,
                                int delay = 10) {
  // Save the previous error for the derivative
  controller.previous_error = controller.error;

  // Calculate a new error by finding the difference between the current
  // position and the desired position.
  // double encoderVal = getEncoderValue(trackingRight, fs);
  if ((useY ? globalY : globalX) > target) {
    controller.error = (useY ? globalY : globalX) - target;
  } else {
    controller.error = target - (useY ? globalY : globalX);
  }
  printf("ER in inches:%f \n", controller.error);
  controller.error = convertToTicks(controller, controller.error);

  Controller1.Screen.setCursor(4, 0);
  Controller1.Screen.print("Error: %5.2f", controller.error);

  // Begin summing the errors into the integral term if the error is below a
  // threshold, and reset it if not. This is to prevent the integral from
  // growing too large.
  double absError =
      (controller.error < 0) ? -1 * controller.error : controller.error;
  if (absError <= 25) {
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
  // 250
  if (absError < 800 && absError > 500) {
    if (power < 0)
      power = -30;
    else
      power = 30;
  } else if (absError <= 500 && absError > 90) {
    if (power < 0)
      power = -10;
    else
      power = 10;
  } else if (absError <= 90 && absError > 50) {
    if (power < 0)
      power = 0; // power = -5;
    else
      power = 5;
  } else if (absError <= 50) {
    if (power < 0)
      power = 0; // power = -3;
    else
      power = 3;
  }

  double gyroVal = gyroInertialSensor.heading(degrees);
  return power;
}

void encoderPrint1(void) {

  float a1 = trackingRight.position(rotationUnits::deg);
  float a2 = trackingRight.position(rotationUnits::deg);
  Brain.Screen.printAt(20, 50, "Left %f Right %f diff %f", a1, a2, (a2 - a1));
  wait(10, msec);
}

void invokePIDDrive(double target, bool useY, double angleOfDirection) {
  encoderPrint1();

  printf("Hello world\n");
  PIDController driveController;
  Brain.resetTimer();

  initalizePIDControllerDrive(driveController, trackingRight, 0.17,
                              0.06, // 0.2, 0.07, 0.01
                              0.01, 100);
  std::ofstream ofs;
  while (true) {
    encoderPrint1();
    double timeElapsed = Brain.timer(timeUnits::sec);
    if (timeElapsed > 4) {
      stopMotorsDrive(true);
      break;
    }

    double power = updatePIDControllerDrive(driveController, target, useY, ofs,
                                            angleOfDirection);
    correctionYes = correctionYes ? false : true;
    if ((useY ? globalY : globalX) < target) {
      setPowerDoubleDrive(power, power, angleOfDirection);
    } else {
      setBackwardPowerDoubleDrive(power, power, angleOfDirection);
    }

    Brain.Screen.printAt(20, 100, "time: %f", Brain.timer(timeUnits::sec));

    spinMotorsDrive();

    Controller1.Screen.setCursor(1, 0);
    Controller1.Screen.print(
        "Encoder: %5.2f",
        convertToInches(driveController,
                        trackingRight.rotation(rotationUnits::deg)));
    wait(100, msec);
    if (power == 0) {
      ofs << "\r\nfile closed beacuse of power";
      printf("\r\nfile closed beacuse of power\n");
      ofs.close();
      Brain.Screen.printAt(50, 40, "file closed");
      encoderPrint1();
      stopMotorsDrive(true);

      break;
    }
  }
}