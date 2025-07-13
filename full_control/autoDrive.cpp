#include"autoDrive.h"

extern bool LockedOn;
extern bool TrackingMode;
extern int target_x;
extern int target_y;
extern int target_w;
extern int target_h;
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
        }
        else
        {
            platform.Fix(target_x, target_y, target_w, target_h);
            platform.PoseSyn();
        }
    }
}