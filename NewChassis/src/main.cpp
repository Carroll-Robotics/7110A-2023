/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\krish                                            */
/*    Created:      Wed Mar 22 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
class Odometer {
  public:
    double prevLeftRotation;
    double prevRightRotation;
    double x;
    double y;
    double ang;
    vex::motor frontLeft = motor(PORT1);
    vex::motor frontRight = motor(PORT1);

    double l = 10;
    Odometer(vex::motor left, vex::motor right) {
      frontLeft = left;
      frontRight = right;
      prevLeftRotation = 0.0;
      prevRightRotation = 0.0;
      ang = 0.0;
      x = 0.0;
      y = 0.0;
    }
    void update() {
      double a = 4*M_PI*((frontLeft.rotation(deg) - prevLeftRotation)/360);
      double b = 4*M_PI*((frontRight.rotation(deg) - prevRightRotation)/360);
      double r = (l*b) / (a-b);
      
      double degre = a/(l+r);
      
      double dist = M_SQRT2 * (l+r);
      
      prevLeftRotation = frontLeft.rotation(deg);
      prevRightRotation = frontRight.rotation(deg);
      
      ang += degre;
      //x += dist*cos(ang);
      //y += dist*sin(ang);
      //Brain.Screen.print("%f %f %f %f %f", x, dist, ang*180/M_PIdegre * 180 / M_PI, x, y);
      Brain.Screen.print("%f %f", degre, ang);

      Brain.Screen.newLine();
    }
  
  
};


using namespace vex;
float maxVelocity = 100;
float robotRadius = 10.25;
float wheelRadius = 2;
float kr = 0.58;
float abs(float f) {
  if(f>0) return f;
  return -f;
}
void setDriveSpeed(float speed) {
  maxVelocity = speed;
  frontLeft.setVelocity(speed, pct);
  frontRight.setVelocity(speed, pct);
}
void leftDrive() {
  float maxLeftSpeed = 1;
  float maxRightSpeed = 0.96;
  while(true) {
    frontLeft.setVelocity((Controller.Axis3.position()+Controller.Axis4.position())*maxLeftSpeed,pct);
    frontRight.setVelocity((Controller.Axis3.position()-Controller.Axis4.position())*maxRightSpeed,pct);
    frontLeft.spin(fwd);
    frontRight.spin(fwd);
    Controller.Screen.newLine();
    Controller.Screen.print(frontLeft.velocity(pct)/frontRight.velocity(pct));
    wait(25,msec);
  }
}
void rightDrive() {
  float maxLeftSpeed = 1;
  float maxRightSpeed = 0.96;
  while(true) {
    frontLeft.setVelocity((Controller.Axis2.position()+Controller.Axis1.position())*maxLeftSpeed,pct);
    frontRight.setVelocity((Controller.Axis2.position()-Controller.Axis1.position())*maxRightSpeed,pct);
    frontLeft.spin(fwd);
    frontRight.spin(fwd);
    Controller.Screen.newLine();
    Controller.Screen.print(frontLeft.velocity(pct)/frontRight.velocity(pct));
    wait(25,msec);
  }
}
void dualDrive() {
  float maxLeftSpeed = 1;
  float maxRightSpeed = 0.96;
  while(true) {
    frontLeft.setVelocity(Controller.Axis3.position()*maxLeftSpeed,pct);
    frontRight.setVelocity(Controller.Axis2.position()*maxRightSpeed,pct);
    frontLeft.spin(fwd);
    frontRight.spin(fwd);
    Controller.Screen.newLine();
    Controller.Screen.print(frontLeft.velocity(pct)/frontRight.velocity(pct));
    wait(25,msec);
  }
}
void splitDrive() {
  float maxLeftSpeed = 1;
  float maxRightSpeed = 0.96;
  while(true) {
    frontLeft.setVelocity((Controller.Axis2.position()+Controller.Axis4.position())*maxLeftSpeed,pct);
    frontRight.setVelocity((Controller.Axis2.position()-Controller.Axis4.position())*maxRightSpeed,pct);
    frontLeft.spin(fwd);
    frontRight.spin(fwd);
    Controller.Screen.newLine();
    Controller.Screen.print("%d  %d",frontLeft.velocity(pct),frontRight.velocity(pct));
    Controller.Screen.newLine();
    wait(25,msec);
  }
}
void simpleDrive(directionType dir, float dist) {
  float rot = dist/(4*M_PI)*360;
  frontLeft.rotateFor(dir, rot, deg,false);
  frontRight.rotateFor(dir, rot, deg,true);
}
bool smartDrive(directionType dir, float dist) {
  float rot = dist/(4*M_PI)*360;
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.75;
  float kd = 0.1;
  float ki = 0.5;
  float dt = 0.05;
  float setpoint = inertialSensor.rotation(deg);
  float currRot = frontRight.rotation(deg);
  frontLeft.setVelocity(maxVelocity,pct);
  frontRight.setVelocity(maxVelocity,pct);
  frontLeft.spin(fwd,maxVelocity,pct);
  frontRight.spin(fwd,maxVelocity,pct);
  wait(dt*4,sec);
  while(frontRight.rotation(deg)-currRot<rot) {
    e = setpoint-inertialSensor.rotation(deg);
    d = (e-eRec)/dt;
    i += e*dt;
    eRec = e;
    frontLeft.setVelocity(frontRight.velocity(rpm) + e*kp + d*kd + i*ki,rpm);
    
    frontLeft.spin(dir);
    frontRight.spin(dir);
    Controller.Screen.newLine();
    Controller.Screen.print("%.2f, %.2f, %.2f", frontRight.rotation(deg), currRot, rot);
    Controller.Screen.newLine();
    wait(dt,sec);
  }
  frontLeft.stop();
  frontRight.stop();
  setDriveSpeed(maxVelocity);
  return true;
}

void simpleTurn(float rot) {
  float wheelRot = kr*rot*robotRadius/wheelRadius;
  frontLeft.rotateFor(fwd, wheelRot, deg,false);
  frontRight.rotateFor(reverse, wheelRot, deg,true);
}
void turnTo(float rot) {
  double currAngle = inertialSensor.rotation();
  float prevSpeed = maxVelocity;
  if(rot>0) {
    for(int i=0;i<1;i++) {
      while(inertialSensor.rotation()-currAngle<rot) {
        frontLeft.spin(fwd);
        frontRight.spin(reverse);
        Controller.Screen.newLine();
        Controller.Screen.print(inertialSensor.rotation());
      }
      /*
      while(inertialSensor.rotation()-currAngle>rot) {
        frontLeft.spin(reverse);
        frontRight.spin(fwd);
        Controller.Screen.newLine();
        Controller.Screen.print(inertialSensor.rotation());
      }
      setDriveSpeed(maxVelocity/2);*/
    }
  } else {
    for(int i=0;i<1;i++) {
      while(inertialSensor.rotation()+currAngle>rot) {
        frontLeft.spin(reverse);
        frontRight.spin(fwd);
        Controller.Screen.newLine();
        Controller.Screen.print(inertialSensor.rotation());
      }
      /*
      while(inertialSensor.rotation()+currAngle<rot) {
        frontLeft.spin(fwd);
        frontRight.spin(reverse);
        Controller.Screen.newLine();
        Controller.Screen.print(inertialSensor.rotation());
      }
      setDriveSpeed(maxVelocity/2);*/
    }
  }
  frontLeft.stop();
  frontRight.stop();
  setDriveSpeed(prevSpeed);
}
bool smartTurn(float rot) {
  float e = 0;
  float d = 0;
  float i = 0;
  float eRec = 0;
  float kp = 0.75;
  float kd = 0;
  float ki = 0;
  float dt = 0.05;
  double currAngle = inertialSensor.rotation(deg);
  double wantedAngle = currAngle + rot;
  e = wantedAngle-inertialSensor.rotation(deg);
  while( abs(e) + abs(d) > 2) {
    e = wantedAngle-inertialSensor.rotation(deg);
    d = (e-eRec)/dt;
    i += e*dt;
    eRec = e;
    float speed = kp*e + kd*d + ki*i;
    frontLeft.setVelocity(speed,pct);
    frontRight.setVelocity(-speed,pct);
    frontLeft.spin(fwd);
    frontRight.spin(fwd);
    
    Controller.Screen.newLine();
    Controller.Screen.print("%.2f %.2f %.2f", e, inertialSensor.rotation(deg), wantedAngle);
    Controller.Screen.newLine();
    wait(dt,sec);
  }
  frontLeft.stop();
  frontRight.stop();
  setDriveSpeed(maxVelocity);
  return true;
}
bool smartTurnTo(float rot) {
  return smartTurn(rot-inertialSensor.heading());
}
void smartSquare(int dist) {
  for(int i=1; i<=4; i++) {
    bool x = smartDrive(fwd, dist);
    waitUntil(x);
    x = smartTurn(90);
    waitUntil(x);
  }
  
}
void stupidSquare(int dist) {
  for(int i=0; i<4; i++) {
    simpleDrive(fwd, dist);
    simpleTurn(90);
  }
}
void safety() {
  while(distanceSensor.value()>250) {
    wait(0.05,sec);
    Controller.Screen.print(distanceSensor.value());
  }
  frontLeft.stop();
  frontRight.stop();
  Controller.Screen.print(0/0);
}
void colors() {
  int col = opticalSensor.hue();
  if(col > 15 && col < 25) { //orange
    simpleTurn(90);
  } else if(col > 110 && col < 120) { //green
    simpleTurn(180);
  } else if(col >300) {
    simpleDrive(reverse, 12);
  }
  Controller.Screen.print(col);
}

//ASHWIN CODE
void turnToHeading(float targetHeading){
  float turnkP = .4;
  float turnkI = .0;
  float turnkD = .0;

  float turnErrorTolerance = 1.0;


  int desiredValue = 200;
  int desiredTurnValue = 0;

  float error;
  float prevError = 0;
  float integral = 0;
  float derivative = 0;
  float timeInterval = 0.025;
  float forwardSpeed = 50;

  error = targetHeading - inertialSensor.rotation(degrees)%360;

  while(abs(error) > 1){
    error = targetHeading - inertialSensor.rotation(degrees)%360;
    integral += error * timeInterval;
    derivative = (error - prevError) / timeInterval;
    float output = ((turnkP * error) + (turnkI * integral) + (derivative * turnkD));
    frontLeft.setVelocity(-output, pct);
    backLeft.setVelocity(-output, pct);
    frontRight.setVelocity(output, pct);
    backRight.setVelocity(output, pct);
    prevError = error;
    frontLeft.spin(forward);
    backLeft.spin(forward);
    frontRight.spin(forward);
    backRight.spin(forward);
    wait(timeInterval, sec);
  }
    frontLeft.stop();
    backLeft.stop();
    frontRight.stop();
    backRight.stop();
}

void driveForward(double targetDistance) {
  float turnkP = .4;
  float turnkI= .2;
  float turnkD = .1;
  float distkP = .15;
  
  float error;
  float prevError = 0;
  float integral = 0;
  float derivative = 0;
  float timeInterval = 0.025;
  float forwardSpeed = 50;

  float traveledDistance = 0;
  float wheelDiameter = 4.0; // inches
  double wheelCircumference = wheelDiameter * M_PI; // inches
  float currentRotation = 0;
  float lastRotation = currentRotation;
  while (traveledDistance < targetDistance) {

    wait(timeInterval, sec);
  }

  // Stop motors and wait
  frontLeft.stop();
  backLeft.stop();
  frontRight.stop();
  backRight.stop();
} 

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  /*inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  }*/
  vexcodeInit();
  setDriveSpeed(50);
  rotationSensor.setReversed(true);
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  }
  //smartDrive(fwd,72);
  //brainTrauma();
  //emergencyStop.broadcast();
  //dualDrive();
  //smartSquare(76);
  //stupidSquare(36);
  /*
  while(true) {
    if(opticalSensor.isNearObject()) {
      colors();
    }
  }*/
  
  Odometer odom(frontLeft,frontRight);
  while(true) {
    odom.update();
    Controller.Screen.print(odom.ang);
    Controller.Screen.clearScreen();
    wait(200,msec);
  }

  

  
}
