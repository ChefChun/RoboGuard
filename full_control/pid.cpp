#include"pid.h"

// 删除全局变量定义，改为extern声明
extern volatile long countLF, countRF, countLB, countRB;
extern float Kp, Ki, Kd;
extern float integralLF, lastErrLF;
extern float integralRF, lastErrRF;
extern float integralLB, lastErrLB;
extern float integralRB, lastErrRB;
extern int targetSpeedLF, targetSpeedRF, targetSpeedLB, targetSpeedRB;
extern int pwmLF, pwmRF, pwmLB, pwmRB;

void turnLeftInPlace(int speed) {
  int pulse = map(speed, 0, 255, 0, 100);
  targetSpeedLF = targetSpeedLB = pulse;
  targetSpeedRF = targetSpeedRB = pulse;
  motorLF.run(BACKWARD);
  motorLB.run(BACKWARD);
  motorRF.run(FORWARD);
  motorRB.run(FORWARD);
}

void turnRightInPlace(int speed) {
  int pulse = map(speed, 0, 255, 0, 100);
  targetSpeedLF = targetSpeedLB = pulse;
  targetSpeedRF = targetSpeedRB = pulse;
  motorLF.run(FORWARD);
  motorLB.run(FORWARD);
  motorRF.run(BACKWARD);
  motorRB.run(BACKWARD);
}

void forwardInPlace(int speed) {
  int pulse = map(speed, 0, 255, 0, 100); // 0~255 PWM映射到0~100脉冲/周期
  targetSpeedLF = targetSpeedRF = targetSpeedLB = targetSpeedRB = pulse;
  motorLF.run(FORWARD);
  motorRF.run(FORWARD);
  motorLB.run(FORWARD);
  motorRB.run(FORWARD);
}

void backwardInPlace(int speed) {
  int pulse = map(speed, 0, 255, 0, 100);
  targetSpeedLF = targetSpeedRF = targetSpeedLB = targetSpeedRB = pulse;
  motorLF.run(BACKWARD);
  motorRF.run(BACKWARD);
  motorLB.run(BACKWARD);
  motorRB.run(BACKWARD);
}

void updatePID() {
  static long lastCountLF = 0, lastCountRF = 0, lastCountLB = 0, lastCountRB = 0;
  static unsigned long lastPID = 0;
  unsigned long now = millis();
  if (now - lastPID < 50) return; // 20Hz
  lastPID = now;

  long speedLF = countLF - lastCountLF;
  long speedRF = countRF - lastCountRF;
  long speedLB = countLB - lastCountLB;
  long speedRB = countRB - lastCountRB;
  lastCountLF = countLF;
  lastCountRF = countRF;
  lastCountLB = countLB;
  lastCountRB = countRB;

  pwmLF = pidCalc(targetSpeedLF, speedLF, &integralLF, &lastErrLF, pwmLF);
  pwmRF = pidCalc(targetSpeedRF, speedRF, &integralRF, &lastErrRF, pwmRF);
  pwmLB = pidCalc(targetSpeedLB, speedLB, &integralLB, &lastErrLB, pwmLB);
  pwmRB = pidCalc(targetSpeedRB, speedRB, &integralRB, &lastErrRB, pwmRB);

  pwmLF = constrain(pwmLF, 0, 255);
  pwmRF = constrain(pwmRF, 0, 255);
  pwmLB = constrain(pwmLB, 0, 255);
  pwmRB = constrain(pwmRB, 0, 255);

  motorLF.setSpeed(pwmLF);
  motorRF.setSpeed(pwmRF);
  motorLB.setSpeed(pwmLB);
  motorRB.setSpeed(pwmRB);
}

int pidCalc(int target, int actual, float* integral, float* lastErr, int lastPWM) {
  float err = target - actual;
  *integral += err;
  float derivative = err - *lastErr;
  float output = Kp * err + Ki * (*integral) + Kd * derivative;
  *lastErr = err;
  return lastPWM + output;
}