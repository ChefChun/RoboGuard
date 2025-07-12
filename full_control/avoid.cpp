#include"avoid.h"

void Avoid() {
  long distL = readDistanceCM(TRIG_PIN_L, ECHO_PIN_L);
  long distM = readDistanceCM(TRIG_PIN_M, ECHO_PIN_M);
  long distR = readDistanceCM(TRIG_PIN_R, ECHO_PIN_R);

  const int SAFE_DIST = 25; // 安全距离cm

  if (distM > 0 && distM < SAFE_DIST) {
    // 前方有障碍
    stopAllMotors();
    delay(100);
    setMotorsBackward(targetSpeed);
    delay(300);
    stopAllMotors();
    delay(100);

    // 判断左右哪边更通畅
    if ((distL > distR && distL > SAFE_DIST) || (distR < SAFE_DIST && distL > SAFE_DIST)) {
      setMotorsTurnLeft(targetSpeed);
      delay(1000);
    } else if ((distR > distL && distR > SAFE_DIST) || (distL < SAFE_DIST && distR > SAFE_DIST)) {
      setMotorsTurnRight(targetSpeed);
      delay(1000);
    } else {
      // 两边都不通，原地多转一会
      setMotorsTurnLeft(targetSpeed);
      delay(2000);
    }
    stopAllMotors();
    delay(100);
  } else if (distL > 0 && distL < SAFE_DIST) {
    // 左侧有障碍，右转
    setMotorsTurnRight(targetSpeed);
    delay(1000);
    stopAllMotors();
    delay(50);
  } else if (distR > 0 && distR < SAFE_DIST) {
    // 右侧有障碍，左转
    setMotorsTurnLeft(targetSpeed);
    delay(1000);
    stopAllMotors();
    delay(50);
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