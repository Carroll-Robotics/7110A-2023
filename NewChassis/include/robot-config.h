using namespace vex;

extern brain Brain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
 extern motor frontRight;
 extern motor backRight;
 extern motor frontLeft;
 extern motor backLeft;
 extern controller Controller;
 extern inertial inertialSensor;
 extern rotation rotationSensor;
 extern distance distanceSensor;
 extern optical opticalSensor;
void vexcodeInit(void);
