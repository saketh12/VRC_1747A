using namespace vex;

extern brain Brain;

// VEXcode devices

extern controller Controller1;
extern motor frontLeftMotor;
extern motor middleLeftMotor;
extern motor middleRightMotor;
extern motor frontRightMotor;
extern motor backRightMotor;
extern motor backLeftMotor;

extern motor intake;
extern motor lift1;
extern pneumatics piston;
extern pneumatics backPiston;

extern encoder trackingLeft;
extern encoder trackingRight;
extern encoder back;
extern inertial gyroInertialSensor;

extern float globalX;
extern float globalY;
extern float globalAngle;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
