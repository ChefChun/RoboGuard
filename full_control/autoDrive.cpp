#include"autoDrive.h"
void autoDrive()
{
    if (TrackingMode == true)
    {
        Avoid();
        PoseAdjust();
    }
    else
    {

    }
}
