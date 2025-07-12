#include"tracking.h"

void PoseAdjust()
{
    String offset = Offset();
    if (offset == "left")
    {
        setMotorsTurnRight(targetSpeed);
        delay(500);
    }
    else if (offset == "right")
    {
        setMotorsTurnLeft(targetSpeed);
        delay(500);
    }
    else
    {
        setMotorsForward(targetSpeed);
    }
}

String Offset()
{
    int ValRight = digitalRead(TrackingPinRight);
    int ValLeft = digitalRead(TrackingPinLeft);
    if (ValRight == LOW && ValLeft == HIGH)
    {
        return "left";
    }
    else if (ValRight == HIGH && ValLeft == LOW)
    {
        return "right";
    }
    else
    {
        return "central";
    }
}

void TrackingDeactivate()
{
    TrackingMode = false;
}

void TrackingActivate()
{
    TrackingMode = true;
}
