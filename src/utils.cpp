#include "vex.h"
using namespace vex;

#include "myUtil.h"

/******************************************************************************************************/
// Contains all the functions for our robot to check the motor temperatures to see if it is overheating.
/** Static Motor Temps
 * - Motor temperatures of all motors
 */
int frontLeftTemp;
int frontRightTemp;
int backLeftTemp;
int backRightTemp;

int lift2LockTemp;
int lift1Temp;

int lift2Temp;

int middleLeftTemp;
int middleRightTemp;
int intakeTemp;

/** Static ALGORITHMIC Variables
 * - Used for Setting Up Controller Rumble
 * - Help with counteracting Overlapping Overheating
 */
static bool overAllHeated = false;
static bool countingDown = false;
static int counter = 100;

/** Static MOTOR Overheating Or Not
 * - Used for testing specific motor
 * - Helps derive individual temperatures of motor
 */
static bool frontLeftOverheated = false;
static bool frontRightOverheated = false;
static bool backLeftOverheated = false;
static bool backRightOverheated = false;

static bool lift2LockOverheated = false;
static bool lift1Overheated = false;

static bool lift2Overheated = false;
static bool middleLeftOverheated = false;
static bool middleRightOverheated = false;
static bool intakeOverheated = false;

/******************************************************************************************************/

void encodersPrint(void) {

  float gyroenc = gyroInertialSensor.rotation();
  float rightenc = trackingRight.position(rotationUnits::deg);
  float backenc = back.position(rotationUnits::deg);
  float leftenc = trackingLeft.position(rotationUnits::deg);
  float lift1enc = lift1.position(rotationUnits::deg);

  Brain.Screen.printAt(20, 50, "Back %f Right %f Left %f", backenc, rightenc,
                       leftenc);
  Brain.Screen.printAt(20, 70, "Gyro %f", gyroenc);
  Brain.Screen.printAt(20, 100, "lift1enc %f", lift1enc);
  wait(10, msec);
}

void printTemperatures(int row, int column, int temp, bool overheated) {
  Controller1.Screen.setCursor(row, column);
  if (!overheated) {
    Controller1.Screen.print(" %d", temp);
  } else {
    Controller1.Screen.print("-%d-", temp);
  }
}

void allPrintTemperatures() {
  // Print Motor Temperatures
  printTemperatures(1, 1, frontLeftTemp, frontLeftOverheated);
  printTemperatures(1, 4, frontRightTemp, frontRightOverheated);
  printTemperatures(1, 7, middleLeftTemp, middleLeftOverheated);
  printTemperatures(2, 1, backLeftTemp, backLeftOverheated);
  printTemperatures(2, 4, backRightTemp, backRightOverheated);
  printTemperatures(2, 7, middleRightTemp, middleRightOverheated);
  printTemperatures(1, 13, lift1Temp, lift1Overheated);

  printTemperatures(3, 14, intakeTemp, intakeOverheated);

  // Print Formatting
  Controller1.Screen.setCursor(3, 2);
  Controller1.Screen.print("-L--R--M");

  Controller1.Screen.setCursor(1, 10);
  Controller1.Screen.print("LF");

  Controller1.Screen.setCursor(2, 9);

  Controller1.Screen.setCursor(3, 11);
  Controller1.Screen.print("I:");
}

void checkForCoolDown() {
  if (frontLeftTemp <= 5) {
    frontLeftOverheated = false;
  }
  if (frontRightTemp <= 5) {
    frontRightOverheated = false;
  }
  if (backLeftTemp <= 5) {
    backLeftOverheated = false;
  }
  if (backRightTemp <= 5) {
    backRightOverheated = false;
  }
  if (lift2LockTemp <= 5) {
    lift2LockOverheated = false;
  }
  if (lift1Temp <= 5) {
    lift1Overheated = false;
  }
  if (lift2Temp <= 5) {
    lift2Overheated = false;
  }
}

void overheating() {
  // Drive
  if (frontLeftTemp > 5 && (!frontLeftOverheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    frontLeftOverheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  if (frontRightTemp > 5 && (!frontRightOverheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    frontRightOverheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  if (backLeftTemp > 5 && (!backLeftOverheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    backLeftOverheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  if (backRightTemp > 5 && (!backRightOverheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    backRightOverheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  // lift2Locks
  if (lift2LockTemp > 5 && (!lift2LockOverheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    lift2LockOverheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  if (lift1Temp > 5 && (!lift1Overheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    lift1Overheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  if (lift2Temp > 5 && (!lift2Overheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    lift2Overheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  if (middleLeftTemp > 5 && (!middleLeftOverheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    middleLeftOverheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  if (middleRightTemp > 5 && (!middleRightOverheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    middleRightOverheated = true;
    overAllHeated = true;
    countingDown = true;
  }

  if (intakeTemp > 5 && (!intakeOverheated && !overAllHeated)) {
    if (!countingDown) {
      Controller1.rumble(".............");
    }
    intakeOverheated = true;
    overAllHeated = true;
    countingDown = true;
  }
}

/******************************************************************************************************/

// Main Temperature Return Method
void showTemperatures() {

  // Controller Screen Reset
  Controller1.Screen.clearScreen();

  // Find the temperatures of the motor
  frontLeftTemp = frontLeftMotor.temperature(percentUnits::pct) / 10;
  frontRightTemp = frontRightMotor.temperature(percentUnits::pct) / 10;
  backLeftTemp = backLeftMotor.temperature(percentUnits::pct) / 10;
  backRightTemp = backRightMotor.temperature(percentUnits::pct) / 10;
  middleRightTemp = middleRightMotor.temperature(percentUnits::pct) / 10;
  middleLeftTemp = middleLeftMotor.temperature(percentUnits::pct) / 10;

  lift1Temp = lift1.temperature(percentUnits::pct) / 10;

  intakeTemp = intake.temperature(percentUnits::pct) / 10;

  // clawTemp = claw.temperature(percentUnits::pct) / 10;

  // Always Print the Motor Temperatures
  allPrintTemperatures();

  // Counting Timer to Avoid Delay
  if (counter == 0) {
    overAllHeated = false;
    countingDown = false;
    counter = 100;
  }

  // Check For Overheating of Motors
  overheating();
  // Print Overheating
  checkForCoolDown();

  // Counts Down After Every Rumble
  if (countingDown) {
    counter--;
  }
}