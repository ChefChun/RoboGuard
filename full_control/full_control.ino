#include <AFMotor.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include"steering_utils.h"
#include"tracking.h"
#include"avoid.h"
#include"pid.h"
#include"autoDrive.h"
#include"Aimer.h"
#include "lcd_manager.h"

Servo horizontal_servo;
Servo plane_servo;

const int FIRING_TIME = 2000; 
int targetSpeed = 120;
const int DEFAULT_SPEED = 120;
String currentCommand = "S";
unsigned long lastStatus = 0;
unsigned long lastAutoAction = 0;
unsigned long current = 0;
bool autoMode = true;
bool TrackingMode = true;
bool LockedOn = false;
const unsigned long statusInterval = 1000; // 1秒
const unsigned long autoInterval = 200;

//These variables should be assigned value according to json file given.
int target_x1 = 0;
int target_y1 = 0;
int target_x2 = 0;
int target_y2 = 0;

int angle = 0;
int score = 0;

// Create an instance of Aimer with pointers
Aimer platform(&horizontal_servo, &plane_servo);

//motors
AF_DCMotor motorLB(1);
AF_DCMotor motorRB(2);
AF_DCMotor motorRF(3);
AF_DCMotor motorLF(4);

//tracking sensor
const int TrackingPinLeft = 39;
const int TrackingPinRight = 35;

//ultrasonic sensor pins
const int TRIG_PIN_L = 26;
const int ECHO_PIN_L = 27;
const int TRIG_PIN_M = 24;
const int ECHO_PIN_M = 25;
const int TRIG_PIN_R = 28;
const int ECHO_PIN_R = 29;

//encoder pins
const int ENCODER_LF = 14; // 左前
const int ENCODER_RF = 15; // 右前
const int ENCODER_LB = 16; // 左后
const int ENCODER_RB = 17; // 右后

//laser pin
const int LASER_PIN = 45;
const int BUZZER_PIN = 34; // 蜂鸣器引脚
const int DIRECTION_THRESHOLD = 5; // 允许的正对误差角度
const int APPROACH_DURATION = 1000; // 靠近目标的时间(ms)
const int BUZZER_DURATION = 1000; // 蜂鸣器和镭射激活时间(ms)

// 唯一全局变量定义区

SystemStatus currentStatus = STATUS_PATROL;
DisplayMode currentMode = MODE_MAIN;
volatile long countLF = 0, countRF = 0, countLB = 0, countRB = 0;
float Kp = 1.2, Ki = 0.05, Kd = 0.1;
float integralLF = 0, lastErrLF = 0;
float integralRF = 0, lastErrRF = 0;
float integralLB = 0, lastErrLB = 0;
float integralRB = 0, lastErrRB = 0;
int targetSpeedLF = 0, targetSpeedRF = 0, targetSpeedLB = 0, targetSpeedRB = 0;
int pwmLF = 0, pwmRF = 0, pwmLB = 0, pwmRB = 0;

// LCD自动切换相关全局变量
unsigned long lastDisplayUpdate = 0;
unsigned long lastModeSwitch = 0;
unsigned long autoSwitchInterval = 3000; // 3秒自动切换

// 全局变量区
int distL = 0, distM = 0, distR = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("System started.");
    pinMode(TrackingPinRight, INPUT);
    pinMode(TrackingPinLeft, INPUT); // Set tracking pin as input
    pinMode(TRIG_PIN_L, OUTPUT);
    pinMode(ECHO_PIN_L, INPUT);
    pinMode(TRIG_PIN_M, OUTPUT);
    pinMode(ECHO_PIN_M, INPUT);
    pinMode(TRIG_PIN_R, OUTPUT);
    pinMode(ECHO_PIN_R, INPUT);
    pinMode(ENCODER_LF, INPUT_PULLUP);
    pinMode(ENCODER_RF, INPUT_PULLUP);
    pinMode(ENCODER_LB, INPUT_PULLUP);
    pinMode(ENCODER_RB, INPUT_PULLUP);
    pinMode(LASER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT); // 初始化蜂鸣器引脚
    digitalWrite(LASER_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);


    //attaching interruptions
    attachInterrupt(digitalPinToInterrupt(ENCODER_LF), encoderLF_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RF), encoderRF_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LB), encoderLB_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RB), encoderRB_ISR, RISING);
    horizontal_servo.attach(9);
    plane_servo.attach(10);
    horizontal_servo.write(45);
    plane_servo.write(90);
    // 集成LCD初始化
    initLCDManager();
}

void loop()
{
    if (Serial.available()){
        processSerialInput();
    }
    current = millis();
    if (current - lastStatus >= statusInterval) {
        sendStatus();
        lastStatus = current;
    }
    if (autoMode && current - lastAutoAction >= autoInterval) {
        if (score > 0) {
            if (angle < -DIRECTION_THRESHOLD) {
                setMotorsTurnLeft(DEFAULT_SPEED / 2);
                Serial.println("外部摄像头指令：左转对准目标");
            } else if (angle > DIRECTION_THRESHOLD) {
                setMotorsTurnRight(DEFAULT_SPEED / 2);
                Serial.println("外部摄像头指令：右转对准目标");
            } else {
                setMotorsForward(DEFAULT_SPEED);
                Serial.println("正对目标，靠近中...");
                delay(APPROACH_DURATION);
                stopAllMotors();
                Serial.println("到达目标，激活镭射和蜂鸣器");
                digitalWrite(LASER_PIN, HIGH);
                digitalWrite(BUZZER_PIN, HIGH);
                delay(BUZZER_DURATION);
                digitalWrite(LASER_PIN, LOW);
                digitalWrite(BUZZER_PIN, LOW);
                score = 0;
            }
        } else {
            autoDrive(); // 无目标时自动驾驶/寻迹/避障
        }
        lastAutoAction = current;
    }
    updatePID();
    updateDisplay();
    distL = readDistanceCM(TRIG_PIN_L, ECHO_PIN_L);
    distM = readDistanceCM(TRIG_PIN_M, ECHO_PIN_M);
    distR = readDistanceCM(TRIG_PIN_R, ECHO_PIN_R);
    setDisplayData(currentStatus, targetSpeed, distL, distM, distR);
}

void processSerialInput() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;
  Serial.print("Command Received: ");
  Serial.println(input);
  if (input.startsWith("{")) {
    parseJsonCommand(input);
  } else {
    handleCommand(input);
  }
}

void parseJsonCommand(String jsonStr) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);
  if (error) {
    Serial.print("JSON parsing error: ");
    Serial.println(error.c_str());
    return;
  }

  // 处理score等AUTO_INFO相关字段
  if (doc.containsKey("score")) {
    score = doc["score"];
    target_x1 = doc["x1"] | 0;
    target_x2 = doc["x2"] | 0;
    target_y1 = doc["y1"] | 0;
    target_y2 = doc["y2"] | 0;
    angle = doc["direction"] | 0;
  }

  // 处理command字段
  if (doc.containsKey("command")) {
    String cmdStr = doc["command"];
    handleCommand(cmdStr);
  }

  // 处理speed字段
  if (doc.containsKey("speed")) {
    int speed = doc["speed"];
    targetSpeed = constrain(speed, 0, 255);
    Serial.print("setting target speed: ");
    Serial.println(targetSpeed);
  }

  Serial.println("JSON command parsed successfully");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  currentCommand = cmd;

  // 新增：警报命令识别
  if (cmd.startsWith("ALERT:")) {
    String alertMsg = cmd.substring(6); // 获取警报内容
    showEmergencyAlert(alertMsg.c_str()); // LCD优先显示警报
    currentStatus = STATUS_ALERT;         // 切换主控状态为警告
    Serial.println("警报已触发: " + alertMsg);
    return;
  }

  // 你可以增加解除警报命令
  if (cmd == "RESUME") {
    currentStatus = STATUS_PATROL;
    Serial.println("恢复巡逻状态");
    return;
  }

  if (cmd == "AUTO") {
    autoMode = true;
    Serial.println("Switching to auto mode");
    return;
  } else {
    autoMode = false;
  }
  if (cmd == "F") {
    setMotorsForward(targetSpeed > 0 ? targetSpeed : DEFAULT_SPEED);
    Serial.println("Moving: Forward");
  } else if (cmd == "B") {
    setMotorsBackward(targetSpeed > 0 ? targetSpeed : DEFAULT_SPEED);
    Serial.println("Moving: Backward");
  } else if (cmd == "L") {
    setMotorsTurnLeft((targetSpeed > 0 ? targetSpeed : DEFAULT_SPEED) / 2);
    Serial.println("Moving: Left");
  } else if (cmd == "R") {
    setMotorsTurnRight((targetSpeed > 0 ? targetSpeed : DEFAULT_SPEED) / 2);
    Serial.println("Moving: Right");
  } else if (cmd == "FL") {
    setMotorsForwardLeft(targetSpeed > 0 ? targetSpeed : DEFAULT_SPEED);
    Serial.println("执行: 左前");
  } else if (cmd == "FR") {
    setMotorsForwardRight(targetSpeed > 0 ? targetSpeed : DEFAULT_SPEED);
    Serial.println("执行: 右前");
  } else if (cmd == "BL") {
    setMotorsBackwardLeft(targetSpeed > 0 ? targetSpeed : DEFAULT_SPEED);
    Serial.println("执行: 左后");
  } else if (cmd == "BR") {
    setMotorsBackwardRight(targetSpeed > 0 ? targetSpeed : DEFAULT_SPEED);
    Serial.println("执行: 右后");
  } else if (cmd == "S") {
    stopAllMotors();
    Serial.println("执行: 停止");
  }
    else {
    Serial.print("Invalid command: ");
    Serial.println(cmd);
  }
}



void sendStatus() {
  StaticJsonDocument<128> doc;
  doc["command"] = currentCommand;
  doc["speed"] = targetSpeed;
  doc["timestamp"] = millis() / 1000;
  doc["score"] = score;
  doc["x1"] = target_x1;
  doc["x2"] = target_x2;
  doc["y1"] = target_y1;
  doc["y2"] = target_y2;
  doc["angle"] = angle;
  serializeJson(doc, Serial);
  Serial.println(); // 换行，便于Python端readline
}


