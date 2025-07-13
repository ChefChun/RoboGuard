#include "Aimer.h"

Aimer :: Aimer(Servo* s1, Servo* s2) {
    x_ori = 90;
    y_ori = 90;
    horizontal = s1;
    plane = s2;
}

void Aimer::setAngle(int x, int y) {

    setHorizontalAngle(y);
    setPlaneAngle(x);
}

void Aimer::setHorizontalAngle(int angle) {
    horizontal->write(angle);
}

void Aimer::setPlaneAngle(int angle) {
    plane->write(angle);
}

void Aimer::PoseSyn()
{
    if (x_ori < 85)
    {
        turnLeftInPlace(TurningSpeed);
    }
    else if (x_ori > 95)
    {
        turnRightInPlace(TurningSpeed);
    }
    else
    {
        forwardInPlace(ForwardSpeed);
    }
}

void Aimer::Scan()
{
    static bool direction_left = true;
    if (direction_left)
    {
        x_ori += 10;
        setPlaneAngle(x_ori);
        if(x_ori == 170)
        {
            direction_left = !direction_left;
        }
    }
    else
    {
        x_ori -= 10;
        setPlaneAngle(x_ori);
        if(x_ori == 10)
        {
            direction_left = !direction_left;
        }
    }
}

void Aimer::Fix(int x, int y, int w, int h)
{
    //Need resolution of the actual image captured by picamera to follow the target.
}