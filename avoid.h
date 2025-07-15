#ifndef AVOIDING
#define AVOIDING
#include <Arduino.h>
#include"pid.h"


extern int targetSpeed;
extern const int TRIG_PIN_L;
extern const int ECHO_PIN_L;
extern const int TRIG_PIN_M;
extern const int ECHO_PIN_M;
extern const int TRIG_PIN_R;
extern const int ECHO_PIN_R;

void Avoid();
long readDistanceCM(int trigPin, int echoPin);

#endif