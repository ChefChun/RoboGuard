#ifndef AUTODRIVE
#define AUTODRIVE
#include"tracking.h"
#include"avoid.h"
#include"Aimer.h"
#include"steering_utils.h"

extern bool LockedOn;
extern bool TrackingMode;
extern int target_x1;
extern int target_y1;
extern int target_x2;
extern int target_y2;
extern int angle;
extern Aimer platform;

void blind_searching();
void autoDrive();
#endif
