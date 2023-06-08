#include "vex.h"
#include "math.h"
using namespace vex;

    
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
    }
    void update() {
      double a = 4*M_PI*((frontLeft.rotation(deg) - prevLeftRotation)/360);
      double b = 4*M_PI*((frontRight.rotation(deg) - prevRightRotation)/360);
      double r = (l*b) / (a-b);
      
      double degre = a/(l+r);
      
      double dist = M_SQRT2 * (l+r);
      
      prevLeftRotation = frontLeft.rotation(deg);
      prevRightRotation = frontRight.rotation(deg);
      
      ang+=degre;/*
      x += dist*cos(ang);
      y += dist*sin(ang);
      Brain.Screen.print("%f %f %f %f %f", x, dist, ang*180/M_PI/*degre * 180 / M_PI, x, y);*/
      Brain.Screen.print(ang);

      Brain.Screen.newLine();
    }
  
  
};