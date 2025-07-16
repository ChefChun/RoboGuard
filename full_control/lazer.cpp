#include"laser.h"


void fire()
{
    digitalWrite(LASER_PIN, HIGH);
    delay(FIRING_TIME);
    digitalWrite(LASER_PIN, LOW);
}