#include "odometry.h"
#include "pidForward.h"
#include "pidGyroTurn.h"
#include "autonomous.h"
#include "myOpControl.h"
#include "utils.h"
#include "robot-config.h"
#include "vex.h"


using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  trackingLeft.setRotation(0, degrees);
  trackingRight.setRotation(0, degrees);
  gyroInertialSensor.setRotation(0, degrees);
  back.setRotation(0, degrees);
  intake.setRotation(0, degrees);
  backPiston.open();
  wait(3, sec);
}

int main() {
  pre_auton();
  Competition.autonomous(leftAuton);
  Competition.drivercontrol(handleRobotDrive);
}