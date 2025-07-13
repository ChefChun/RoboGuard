#ifndef TRACKING
#define TRACKING
#include"pid.h"
#include <Arduino.h>

extern bool TrackingMode;
extern int targetSpeed;
extern const int TrackingPinLeft;
extern const int TrackingPinRight;

void PoseAdjust();
String Offset();
void TrackingDeactivate();
void TrackingActivate();

#endif