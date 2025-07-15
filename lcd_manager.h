#ifndef LCD_MANAGER_H
#define LCD_MANAGER_H

#include "lcd_display.h"
#include "pid.h"

// 注意：DisplayMode 已在 lcd_display.h 中定义，这里不再重复定义
// 使用 lcd_display.h 中的 DisplayMode 枚举

// 显示优先级
enum DisplayPriority {
    PRIORITY_LOW,    // 低优先级
    PRIORITY_NORMAL, // 普通优先级
    PRIORITY_HIGH,   // 高优先级
    PRIORITY_CRITICAL // 关键优先级（警报等）
};

// 初始化LCD管理器
void initLCDManager();

// 显示管理函数
void updateDisplay();
void setDisplayMode(DisplayMode mode);
void switchDisplayMode();

// 主界面显示
void showMainDisplay();
void showMainDisplayWithData(SystemStatus status, int speed);

// 主界面显示
void showMainScreen(SystemStatus status, int speed);
const char* getStatusText(SystemStatus status);

// 传感器显示
void showSensorDisplay();
void showSensorDisplayWithData(int distL, int distM, int distR);

// 速度显示（只显示目标速度）
void showSpeedDisplay();
void showSpeedDisplayWithData(int targetSpeed, int actualSpeed);

// 电池显示
void showBatteryDisplay();
void showBatteryDisplayWithData(int batteryLevel);

// 菜单显示
void showMenuDisplay();
void showMenuSelection(int selection, int total);

// 警报显示
void showAlertDisplay(const char* message);
void showErrorDisplay(const char* error, int code);

// 自动切换显示
void autoSwitchDisplay();
void setAutoSwitchInterval(unsigned long interval);

// 紧急显示
void showEmergencyAlert(const char* message);
void clearEmergencyAlert();

// 状态监控
void updateDisplayData();
void setDisplayData(SystemStatus status, int speed, int distL, int distM, int distR);

#endif 