#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller Controller = controller(primary);
motor frontLeft = motor(PORT19, ratio18_1, false);
motor backLeft = motor(PORT20, ratio18_1, false);
motor frontRight = motor(PORT2, ratio18_1, true);
motor backRight = motor(PORT12, ratio18_1, true);
inertial inertialSensor = inertial(PORT15);
rotation rotationSensor = rotation(PORT16);
//distance distanceSensor = distance(PORT4);
//optical opticalSensor = optical(PORT4);

void vexcodeInit(void) {
  // Nothing to initialize
}