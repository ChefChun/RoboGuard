#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// I2C LCD地址（常见为0x27或0x3F）
#define LCD_I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// 显示模式枚举 - 针对16x2优化
enum DisplayMode {
    MODE_MAIN,      // 主界面：状态+速度
    MODE_SENSORS,   // 传感器：距离信息
    MODE_SPEED,     // 速度：目标+实际
    MODE_BATTERY,   // 电池：电量状态
    MODE_MENU,      // 菜单：选项选择
    MODE_ALERT,     // 警报：紧急信息
    MODE_DEBUG      // 调试信息
};

// 系统状态枚举
enum SystemStatus {
    STATUS_IDLE,
    STATUS_PATROL,
    STATUS_TRACKING,
    STATUS_AVOIDING,
    STATUS_ALERT,
    STATUS_ERROR
};

// 初始化LCD
void initLCD();

// 基础显示函数
void clearDisplay();
void setCursor(int col, int row);
void printText(const char* text);
void printText(int col, int row, const char* text);

// 状态显示 - 针对16x2优化
void showStatus(SystemStatus status);
void showStatusWithSpeed(SystemStatus status, int speed);
void showStatusWithDistance(SystemStatus status, int distance);

// 距离显示 - 一行显示三个距离
void showDistance(int distL, int distM, int distR);
void showDistanceSimple(int distance);

// 速度显示 - 只显示目标速度
void showSpeed(int targetSpeed, int actualSpeed);
void showSpeedAll(int speedLF, int speedRF, int speedLB, int speedRB);

// 菜单显示 - 简洁的菜单系统
void showMenu(const char* title, const char* option1, const char* option2);
void showMenuSelection(const char* title, int selection, int total);

// 警报显示 - 突出显示警报信息
void showAlert(const char* alert);
void showAlertWithCode(const char* alert, int code);
void showBatteryLow(int batteryLevel);

// 电池显示 - 简洁的电池状态
void showBattery(int batteryLevel);
void showBatteryWithIcon(int batteryLevel);

// 调试显示 - 显示关键调试信息
void showDebug(const char* info, int value);
void showDebugTwoValues(int val1, int val2);

// 滚动显示 - 长文本滚动
void scrollText(const char* text, int row);
void scrollTextSimple(const char* text);

// 自定义字符显示
void createCustomChar(byte location, byte charmap[]);
void showCustomChar(byte location, int col, int row);

// 动画效果 - 简单的动画
void showLoading();
void showPatrol();
void showAlertBlink();

// 组合显示 - 常用组合
void showMainScreen(SystemStatus status, int speed);
const char* getStatusText(SystemStatus status);
void showSensorScreen(int distL, int distM, int distR, bool tracking);

// 格式化函数 - 针对16字符限制
String formatDistance(int distance);
String formatSpeed(int speed);
String formatBattery(int battery);
String formatStatus(SystemStatus status);

#endif 