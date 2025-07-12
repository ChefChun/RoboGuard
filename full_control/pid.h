#ifndef PIDCONTROLL
#define PIDCONTROLL
#include"steering_utils.h"
#include <Arduino.h>

extern float Kp, Ki, Kd;
extern float integralLF, lastErrLF;
extern float integralRF, lastErrRF;
extern float integralLB, lastErrLB;
extern float integralRB, lastErrRB;

extern AF_DCMotor motorLF;  // 左前轮 - M1
extern AF_DCMotor motorRF;  // 右前轮 - M2  
extern AF_DCMotor motorLB;  // 左后轮 - M3
extern AF_DCMotor motorRB;  // 右后轮 - M4

extern volatile long countLF;
extern volatile long countRF;
extern volatile long countLB;
extern volatile long countRB;

extern int targetSpeedLF;
extern int targetSpeedRF;
extern int targetSpeedLB;
extern int targetSpeedRB;
extern int pwmLF;
extern int pwmRF;
extern int pwmLB;
extern int pwmRB;


void turnLeftInPlace(int speed);
void turnRightInPlace(int speed);
void forwardInPlace(int speed);
void backwardInPlace(int speed);
void updatePID();
int pidCalc(int target, int actual, float* integral, float* lastErr, int lastPWM);
void encoderLF_ISR();
void encoderRF_ISR();
void encoderLB_ISR();
void encoderRB_ISR();

#endif