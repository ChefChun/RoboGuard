#include"autoDrive.h"

extern bool LockedOn;
extern bool TrackingMode;
extern int target_x1;
extern int target_y1;
extern int target_x2;
extern int target_y2;
extern Aimer platform;

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