#include "Aimer.h"
Aimer :: Aimer(Servo* s1, Servo* s2) {
    x_ori = 90;
    y_ori = 45;
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
    Serial.print("firing");
    y_ori = 0;
    setHorizontalAngle(y_ori);
    fire();
    TrackingMode = true;
    LockedOn = false;
    target_x1 = 0;
    target_x2 = 0;
    target_y1 = 0;
    target_y2 = 0;
}
