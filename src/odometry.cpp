#include "odometry.h"

#include "vex.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>

// Contains all the code for our custom odometry algorithm, which accurately tracks the x, y, and angle positions of our robot anywhere on the field. 
// It works accurately with any type of motion the robot may make. 

double lastLeft = 0;
double lastRight = 0;
double lastBack = 0;

double convertToInches(double val) {
  double revs = val / 360.0;
  const double diameter = 2.75;
  double circumference = M_PI * diameter;
  return (revs * circumference);
}
double convertToDegrees(double val) {
  const double change = 180.0 / M_PI;
  double deg = change * val;
  return deg;
}
double toRadians(double val) {
  const double change = M_PI / 180.0;
  double deg = change * val;
  return deg;
}
double toRadiansFloat(float val) {
  const float change = M_PI / 180.0;
  double deg = change * val;
  return deg;
}
long double round(long double var) {
  double value = (int)(var * 1000000 + .5);
  return (double)value / 1000000;
}
int odomAutonomousSK(void) {

  // AUTON - **********************
  trackingLeft.setRotation(0, degrees);
  trackingRight.setRotation(0, degrees);
  back.setRotation(0, degrees);
  lastLeft = 0;
  lastRight = 0;
  lastBack = 0;
  double angleLastCycle = 0;
  const double width = 9.5; 
  double lastTheta = 0.0;
  while (true) {
    double curLeftPosDeg = (double)(-trackingLeft.rotation(degrees));
    double curRightPosDeg = (double)(-trackingRight.rotation(degrees));
    double curBackPosDeg = (double)(back.rotation(degrees));

    double changeLeft =
        convertToInches(curLeftPosDeg) - convertToInches(lastLeft);
    double changeRight =
        convertToInches(curRightPosDeg) - convertToInches(lastRight);
    double changeBack =
        convertToInches(curBackPosDeg) - convertToInches(lastBack);

    lastLeft = curLeftPosDeg;
    lastRight = curRightPosDeg;
    lastBack = curBackPosDeg;
    double curTheta = (double)(toRadians(gyroInertialSensor.rotation()));
    double changeTheta = curTheta - lastTheta;
    double dis = (width / 2.0) + (changeRight / changeTheta);
    double h = 2 * sin(changeTheta / 2) * dis;
    double shiftAngle = lastTheta + (changeTheta / 2.0);
    double disBack = (6.0) + (changeBack / changeTheta);
    double hBack = 2 * sin(changeTheta / 2) * disBack;
    long double changeY =
        (cos(shiftAngle) * h) + (-1) * (sin(shiftAngle) * hBack);

    long double changeB = (cos(shiftAngle) * hBack) + (sin(shiftAngle) * h);

    changeB = round(changeB);
    changeY = round(changeY);
    globalX += changeB;
    globalY += changeY;
    Brain.Screen.printAt(20, 50, "Global X: %f Global Y: %f", globalX, globalY);
    Brain.Screen.printAt(20, 70, "Global Angle %f", globalAngle);
    angleLastCycle =
        ((convertToInches(curLeftPosDeg) - convertToInches(curRightPosDeg)) /
         width);
    lastTheta = curTheta;
    globalAngle = (float)(gyroInertialSensor.rotation());
    wait(10, msec);
  }

  return 0;
}

void reset() {
  trackingLeft.setRotation(0, degrees);
  trackingRight.setRotation(0, degrees);
  back.setRotation(0, degrees);
  gyroInertialSensor.resetRotation();
  lastBack = 0;
  lastRight = 0;
  lastLeft = 0;
  globalX = 0;
  globalY = 0;
  globalAngle = 0;
}
double getX() { return globalX; }
double getY() { return globalY; }
double getAngleDegrees() { return globalAngle; }
