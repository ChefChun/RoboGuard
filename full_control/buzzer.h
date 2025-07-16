#ifndef BUZZER_H
#define BUZZER_H

void buzzerInit(int pin);
void buzzerAlarm(int repeatCount = 10); // repeatCount为警报循环次数，可选参数

#endif
