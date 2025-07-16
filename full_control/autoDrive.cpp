#include"autoDrive.h"


const int TURN_SPEED = 255;
const int BACK_SPEED = 200;
const int BACK_TIME = 400;   // ms
const int TURN_TIME = 900;   // ms
const int PAUSE_TIME = 40;  // ms

void autoDrive()
{
    if (TrackingMode)
    {
        Avoid();
        PoseAdjust();
        // platform.Scan(); // 禁用本地摄像头扫描
    }
    else
    {
        Avoid();
        if (!LockedOn)
        {
           // blind_searching();
           // platform.Scan();
        }
        else
        {
            platform.Fix(target_x1, target_y1, target_x2, target_y2);
            platform.PoseSyn();
        }
    }
}

// void blind_searching() { ... } // 可保留但不再被调用