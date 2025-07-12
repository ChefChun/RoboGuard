#include <AFMotor.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include"steering_utils.h"
#include"tracking.h"
#include"avoid.h"
#include"pid.h"

Servo horizontal_servo;
Servo plane_servo;

int targetSpeed = 120;
const int DEFAULT_SPEED = 120;
String currentCommand = "S";
unsigned long lastStatus = 0;
unsigned long current = 0;
bool autoMode = false;
bool TrackingMode = false;
const unsigned long statusInterval = 1000; // 1秒


class Aimer {
    Servo* horizontal;
    Servo* plane;

public:
    Aimer(Servo* s1, Servo* s2) {
        horizontal = s1;
        plane = s2;
    }

    void setAngle(int x, int y) {
        setHorizontalAngle(y);
        setPlaneAngle(x);
    }

    void setHorizontalAngle(int angle) {
        horizontal->write(angle);
    }

    void setPlaneAngle(int angle) {
        plane->write(angle);
    }
};

// Create an instance of Aimer with pointers
Aimer platform(&horizontal_servo, &plane_servo);

//motors
AF_DCMotor motorLB(1);
AF_DCMotor motorRB(2);
AF_DCMotor motorRF(3);
AF_DCMotor motorLF(4);

//tracking sensor
const int TrackingPinLeft = 35;
const int TrackingPinRight = 36;

//ultrasonic sensor pins
const int TRIG_PIN_L = 26;
const int ECHO_PIN_L = 27;
const int TRIG_PIN_M = 24;
const int ECHO_PIN_M = 25;
const int TRIG_PIN_R = 28;
const int ECHO_PIN_R = 29;

//encoder pins
const int ENCODER_LF = 18; // 左前
const int ENCODER_RF = 19; // 右前
const int ENCODER_LB = 20; // 左后
const int ENCODER_RB = 21; // 右后

volatile long countLF = 0, countRF = 0, countLB = 0, countRB = 0;

//PID parameters
float Kp = 1.2, Ki = 0.05, Kd = 0.1;
float integralLF = 0, lastErrLF = 0;
float integralRF = 0, lastErrRF = 0;
float integralLB = 0, lastErrLB = 0;
float integralRB = 0, lastErrRB = 0;

int targetSpeedLF = 0, targetSpeedRF = 0, targetSpeedLB = 0, targetSpeedRB = 0;
int pwmLF = 0, pwmRF = 0, pwmLB = 0, pwmRB = 0;

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
    //attaching interruptions
    attachInterrupt(digitalPinToInterrupt(ENCODER_LF), encoderLF_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RF), encoderRF_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LB), encoderLB_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RB), encoderRB_ISR, RISING);
}

void loop()
{
    if (Serial.available()) {
        processSerialInput();
        if (autoMode) {
            // autoDrive();
            //here autoDrive is a combination of avoiding and tracking
            //not done yet
        }
    }
    current = millis();
    if (current - lastStatus >= statusInterval) {
        sendStatus();
        lastStatus = current;
    }
    updatePID();
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
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);
  if (error) {
    Serial.print("JSON parsing error: ");
    Serial.println(error.c_str());
    return;
  }
  if (doc.containsKey("command")) {
    String cmdStr = doc["command"];
    handleCommand(cmdStr);
  }
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
  if (cmd == "AUTO") {
    autoMode = true;
    Serial.println("Switching to auto mode");
    return;
  } else {
    autoMode = false; // 任何手动命令都退出自动驾驶
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
  serializeJson(doc, Serial);
  Serial.println(); // 换行，便于Python端readline
}


