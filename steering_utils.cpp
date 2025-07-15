#include"steering_utils.h"

extern AF_DCMotor motorLB;
extern AF_DCMotor motorRB;
extern AF_DCMotor motorRF;
extern AF_DCMotor motorLF;

void setMotorsForward(int speed) {
  motorLF.setSpeed(speed);
  motorRF.setSpeed(speed);
  motorLB.setSpeed(speed);
  motorRB.setSpeed(speed);
  motorLF.run(FORWARD);
  motorRF.run(FORWARD);
  motorLB.run(FORWARD);
  motorRB.run(FORWARD);
}

void setMotorsBackward(int speed) {
  motorLF.setSpeed(speed);
  motorRF.setSpeed(speed);
  motorLB.setSpeed(speed);
  motorRB.setSpeed(speed);
  motorLF.run(BACKWARD);
  motorRF.run(BACKWARD);
  motorLB.run(BACKWARD);
  motorRB.run(BACKWARD);
}

void setMotorsTurnLeft(int speed) {
  int innerSpeed = speed * 1 / 3; // 左侧低速
  int outerSpeed = speed;
  motorLF.setSpeed(innerSpeed);
  motorLB.setSpeed(outerSpeed);
  motorRF.setSpeed(outerSpeed);
  motorRB.setSpeed(outerSpeed);
  motorLF.run(FORWARD);
  motorLB.run(BACKWARD);
  motorRF.run(FORWARD);
  motorRB.run(FORWARD);
}

void setMotorsTurnRight(int speed) {
  int innerSpeed = speed * 1 / 3; // 右侧低速
  int outerSpeed = speed;
  motorLF.setSpeed(outerSpeed);
  motorLB.setSpeed(outerSpeed);
  motorRF.setSpeed(innerSpeed);
  motorRB.setSpeed(outerSpeed);
  motorLF.run(FORWARD);
  motorLB.run(FORWARD);
  motorRF.run(FORWARD);
  motorRB.run(BACKWARD);
}

void setMotorsForwardLeft(int speed) {
  // 左前：左轮半速，右轮全速前进
  motorLF.setSpeed(speed / 2);
  motorLB.setSpeed(speed / 2);
  motorRF.setSpeed(speed);
  motorRB.setSpeed(speed);
  motorLF.run(FORWARD);
  motorLB.run(FORWARD);
  motorRF.run(FORWARD);
  motorRB.run(FORWARD);
}

void setMotorsForwardRight(int speed) {
  // 右前：右轮半速，左轮全速前进
  motorLF.setSpeed(speed);
  motorLB.setSpeed(speed);
  motorRF.setSpeed(speed / 2);
  motorRB.setSpeed(speed / 2);
  motorLF.run(FORWARD);
  motorLB.run(FORWARD);
  motorRF.run(FORWARD);
  motorRB.run(FORWARD);
}

void setMotorsBackwardLeft(int speed) {
  // 左后：左轮半速，右轮全速后退
  motorLF.setSpeed(speed / 2);
  motorLB.setSpeed(speed / 2);
  motorRF.setSpeed(speed);
  motorRB.setSpeed(speed);
  motorLF.run(BACKWARD);
  motorLB.run(BACKWARD);
  motorRF.run(BACKWARD);
  motorRB.run(BACKWARD);
}

void setMotorsBackwardRight(int speed) {
  // 右后：右轮半速，左轮全速后退
  motorLF.setSpeed(speed);
  motorLB.setSpeed(speed);
  motorRF.setSpeed(speed / 2);
  motorRB.setSpeed(speed / 2);
  motorLF.run(BACKWARD);
  motorLB.run(BACKWARD);
  motorRF.run(BACKWARD);
  motorRB.run(BACKWARD);
}

void stopAllMotors() {
  motorLF.run(RELEASE);
  motorRF.run(RELEASE);
  motorLB.run(RELEASE);
  motorRB.run(RELEASE);
}