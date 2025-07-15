#include"autoDrive.h"


const int TURN_SPEED = 255;
const int BACK_SPEED = 200;
const int BACK_TIME = 400;   // ms
const int TURN_TIME = 900;   // ms
const int PAUSE_TIME = 40;  // ms

void autoDrive()
{
    if (TrackingMode == true)
    {
        Avoid();
        PoseAdjust();
    }
    else
    {
        Avoid();
        if (!LockedOn)
        {
            blind_searching();
            platform.Scan();
            //if scanned, then LockedOn == true
        }
        else
        {
            platform.Fix(target_x1, target_y1, target_x2, target_y2);
            platform.PoseSyn();
        }
    }
}

void blind_searching()
{
    if (target_x1 != target_x2)
    {
        LockedOn = true;
    }
    else if (0< angle <= 2)
    {
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      turnRightInPlace(TURN_SPEED);
      delay(TURN_TIME);
    }
    else if (2< angle <= 4)
    {
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      turnRightInPlace(TURN_SPEED);
      delay(TURN_TIME);

      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      turnRightInPlace(TURN_SPEED);
      delay(TURN_TIME);
    }
    else if (4 < angle <= 6)
    {
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      turnLeftInPlace(TURN_SPEED);
      delay(TURN_TIME);

      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      turnLeftInPlace(TURN_SPEED);
      delay(TURN_TIME);
    }
    else
    {
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      turnLeftInPlace(TURN_SPEED);
      delay(TURN_TIME);
    }
}