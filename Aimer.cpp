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
    static bool direction_down = true;
    if (direction_left)
    {
        x_ori += 10;
        setPlaneAngle(x_ori);
        if(x_ori >= 170)
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

    if (direction_down)
    {
        y_ori += 5;
        setHorizontalAngle(y_ori);
        if(y_ori >= 45)
        {
            direction_down = !direction_down;
        }
    }
    else
    {
        y_ori -= 5;
        setHorizontalAngle(y_ori);
        if(y_ori == 0)
        {
            direction_down = !direction_down;
        }
    }
}

void Aimer::Fix(int x1, int y1, int x2, int y2)
{
    //image quality of 640*480
    //Need resolution of the actual image captured by picamera to follow the target.
    if (320 > x2 * 3/5 + x1 * 2/5)
    {
        x_ori += 5;
        setPlaneAngle(x_ori);
    }
    else if (320 < x1 * 3/5 + x2 * 2/5)
    {
        x_ori -= 5;
        setPlaneAngle(x_ori);
    }

    if (y2 >= 420)
    {
        y_ori += 5;
        setHorizontalAngle(y_ori);
    }
}