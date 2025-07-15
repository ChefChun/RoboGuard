#include "buzzer.h"
#include <Arduino.h>

static int buzzerPin = -1;

void buzzerInit(int pin) {
    buzzerPin = pin;
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
}

void buzzerAlarm(int repeatCount) {
    if (buzzerPin == -1) return; // 未初始化
    for (int i = 0; i < repeatCount; i++) {
        tone(buzzerPin, 1000); // 高音
        delay(150);
        tone(buzzerPin, 2000); // 更高音
        delay(150);
        noTone(buzzerPin);
        delay(50);
    }
}
