#include "lcd_manager.h"

// 删除全局变量定义，改为extern声明
extern DisplayMode currentMode;
extern SystemStatus currentStatus;
extern DisplayPriority currentPriority;
extern unsigned long lastDisplayUpdate;
extern unsigned long lastModeSwitch;
extern unsigned long autoSwitchInterval; // 3秒自动切换

// 显示数据
struct DisplayData {
    SystemStatus status = STATUS_IDLE;
    int speed = 0;
    int distL = 0, distM = 0, distR = 0;
    int targetSpeed = 0;
    int actualSpeed = 0;
} displayData;

// 紧急警报
bool emergencyAlert = false;
String emergencyMessage = "";
unsigned long alertStartTime = 0;
const unsigned long alertDuration = 5000; // 5秒警报显示

// 初始化LCD管理器
void initLCDManager() {
    initLCD();
    
    // 显示启动信息
    showMainDisplayWithData(STATUS_IDLE, 0);
    
    Serial.println("LCD Manager initialized");
}

// 显示管理函数
void updateDisplay() {
    unsigned long now = millis();
    
    // 检查紧急警报
    if (emergencyAlert) {
        if (now - alertStartTime < alertDuration) {
            showAlertDisplay(emergencyMessage.c_str());
            return;
        } else {
            emergencyAlert = false;
        }
    }
    
    // 自动切换显示模式
    if (now - lastModeSwitch >= autoSwitchInterval) {
        autoSwitchDisplay();
        lastModeSwitch = now;
    }
    
    // 更新显示
    if (now - lastDisplayUpdate >= 500) { // 500ms更新间隔
        switch (currentMode) {
            case MODE_MAIN:
                showMainDisplayWithData(displayData.status, displayData.speed);
                break;
            case MODE_SENSORS:
                showSensorDisplayWithData(displayData.distL, displayData.distM, displayData.distR);
                break;
            case MODE_SPEED:
                showSpeedDisplayWithData(displayData.targetSpeed, displayData.actualSpeed);
                break;
            case MODE_ALERT:
                showAlertDisplay(emergencyMessage.c_str());
                break;
        }
        lastDisplayUpdate = now;
    }
}

void setDisplayMode(DisplayMode mode) {
    currentMode = mode;
    lastModeSwitch = millis(); // 重置自动切换计时器
}

void switchDisplayMode() {
    // 循环切换显示模式
    switch (currentMode) {
        case MODE_MAIN:
            setDisplayMode(MODE_SENSORS);
            break;
        case MODE_SENSORS:
            setDisplayMode(MODE_SPEED);
            break;
        case MODE_SPEED:
            setDisplayMode(MODE_MAIN);
            break;
        default:
            setDisplayMode(MODE_MAIN);
            break;
    }
}

// 主界面显示
void showMainDisplay() {
    showMainDisplayWithData(displayData.status, displayData.speed);
}

void showMainDisplayWithData(SystemStatus status, int speed) {
    showMainScreen(status, speed);
}

// 传感器显示
void showSensorDisplay() {
    showSensorDisplayWithData(displayData.distL, displayData.distM, displayData.distR);
}

void showSensorDisplayWithData(int distL, int distM, int distR) {
    showDistance(distL, distM, distR);
}

// 速度显示
void showSpeedDisplay() {
    showSpeedDisplayWithData(displayData.targetSpeed, displayData.actualSpeed);
}

void showSpeedDisplayWithData(int targetSpeed, int actualSpeed) {
    showSpeed(targetSpeed, actualSpeed);
}

// 菜单显示
void showMenuDisplay() {
    showMenu("Main Menu", "Status", "Settings");
}

void showMenuSelection(int selection, int total) {
    showMenuSelection("Menu", selection, total);
}

// 警报显示
void showAlertDisplay(const char* message) {
    showAlert(message); // 由lcd_display.cpp完善详细报告
}

void showErrorDisplay(const char* error, int code) {
    showAlertWithCode(error, code);
}

// 自动切换显示
void autoSwitchDisplay() {
    if (currentMode != MODE_ALERT && !emergencyAlert) {
        switchDisplayMode();
    }
}

void setAutoSwitchInterval(unsigned long interval) {
    autoSwitchInterval = interval;
}

// 紧急显示
void showEmergencyAlert(const char* message) {
    emergencyAlert = true;
    emergencyMessage = String(message);
    alertStartTime = millis();
    showAlertDisplay(message);
}

void clearEmergencyAlert() {
    emergencyAlert = false;
}

// 状态监控
void updateDisplayData() {
    // 更新显示数据
    displayData.actualSpeed = (pwmLF + pwmRF + pwmLB + pwmRB) / 4;
    displayData.targetSpeed = targetSpeedLF; // 使用左前轮作为目标速度代表
}

void setDisplayData(SystemStatus status, int speed, int distL, int distM, int distR) {
    displayData.status = status;
    displayData.speed = speed;
    displayData.distL = distL;
    displayData.distM = distM;
    displayData.distR = distR;
    displayData.targetSpeed = speed;
}

// 辅助函数：获取当前显示模式名称
String getDisplayModeName(DisplayMode mode) {
    switch (mode) {
        case MODE_MAIN: return "Main";
        case MODE_SENSORS: return "Sensors";
        case MODE_SPEED: return "Speed";
        case MODE_BATTERY: return "Battery";
        case MODE_MENU: return "Menu";
        case MODE_ALERT: return "Alert";
        default: return "Unknown";
    }
}

// 辅助函数：检查是否需要显示警报
bool shouldShowAlert() {
    // 检查距离警报
    if (displayData.distM < 20 || displayData.distL < 15 || displayData.distR < 15) {
        return true;
    }
    
    // 检查系统状态
    if (displayData.status == STATUS_ALERT || displayData.status == STATUS_ERROR) {
        return true;
    }
    
    return false;
} 