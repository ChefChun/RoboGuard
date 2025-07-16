#include"tracking.h"

void PoseAdjust()
{
    String offset = Offset();
    if (offset == "left")
    {
        stopAllMotors();
        delay(120);
        turnRightInPlace(255);
        delay(900);
    }
    else if (offset == "right")
    {
        stopAllMotors();
        delay(120);
        turnLeftInPlace(255);
        delay(900);
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
