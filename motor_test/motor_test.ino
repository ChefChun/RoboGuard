#include <Arduino.h>

const int MotorPin = 47;

void setup()
{
    pinMode(MotorPin, OUTPUT);
}
void loop()
{
    digitalWrite(MotorPin, HIGH);
}