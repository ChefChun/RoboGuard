#ifndef AIMING
#define AIMING
#include <Arduino.h>
#include "Servo.h"
#include "pid.h"

static int TurningSpeed = 200;
static int ForwardSpeed = 200;

class Aimer {
    Servo* horizontal;
    Servo* plane;
    int x_ori;
    int y_ori;

public:
    Aimer(Servo* s1, Servo* s2);
    void setAngle(int x, int y);
    void setHorizontalAngle(int angle);
    void setPlaneAngle(int angle);
    void PoseSyn();
    void Scan();
    void Fix(int x, int y, int w, int h);
};


#endif