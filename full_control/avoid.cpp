#include"avoid.h"

void Avoid() {
  long distL = readDistanceCM(TRIG_PIN_L, ECHO_PIN_L);
  long distM = readDistanceCM(TRIG_PIN_M, ECHO_PIN_M);
  long distR = readDistanceCM(TRIG_PIN_R, ECHO_PIN_R);

  const int SAFE_DIST = 35; // 安全距离cm

  const int TURN_SPEED = 255;
  const int BACK_SPEED = 200;
  const int BACK_TIME = 400;   // ms
  const int TURN_TIME = 900;   // ms
  const int PAUSE_TIME = 40;  // ms

  if (distM > 0 && distM < SAFE_DIST) {
    // 前方有障碍
    stopAllMotors();
    delay(PAUSE_TIME);
    backwardInPlace(targetSpeed);
    delay(BACK_TIME);
    stopAllMotors();
    delay(PAUSE_TIME);

    // 判断左右哪边更通畅
    if ((distL > distR && distL > SAFE_DIST) || (distR < SAFE_DIST && distL > SAFE_DIST)) {
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      turnLeftInPlace(TURN_SPEED);
      delay(TURN_TIME);
    } else if ((distR > distL && distR > SAFE_DIST) || (distL < SAFE_DIST && distR > SAFE_DIST)) {
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      turnRightInPlace(TURN_SPEED);
      delay(TURN_TIME);
    } else {
      // 两边都不通，原地多转一会
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
      setMotorsTurnLeft(TURN_SPEED);
      delay(TURN_TIME * 1.5);
    }
    stopAllMotors();
    delay(PAUSE_TIME);
  } else if (distL > 0 && distL < SAFE_DIST) {
    // 左侧有障碍，右转
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
    setMotorsTurnRight(TURN_SPEED);
    delay(TURN_TIME);
    stopAllMotors();
    delay(PAUSE_TIME);
  } else if (distR > 0 && distR < SAFE_DIST) {
    // 右侧有障碍，左转
      stopAllMotors();
      delay(PAUSE_TIME);
      backwardInPlace(targetSpeed * 3 / 5);
      delay(BACK_TIME);
    setMotorsTurnLeft(TURN_SPEED);
    delay(TURN_TIME);
    stopAllMotors();
    delay(PAUSE_TIME);
  } else {
    // 前方畅通
    setMotorsForward(targetSpeed);
  }
} 

long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms超时
  long distance = duration * 0.034 / 2;
  return distance;
}