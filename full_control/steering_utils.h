#ifndef STEERING
#define STEERING
#include <Arduino.h>
#include <AFMotor.h>

void setMotorsForward(int speed);
void setMotorsBackward(int speed);
void setMotorsTurnLeft(int speed);
void setMotorsTurnRight(int speed);
void setMotorsForwardLeft(int speed);
void setMotorsForwardRight(int speed);
void setMotorsBackwardLeft(int speed);
void setMotorsBackwardRight(int speed);
void stopAllMotors();

#endif