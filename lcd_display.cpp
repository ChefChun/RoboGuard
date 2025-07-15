#include "lcd_display.h"

// 创建I2C LCD对象
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// 删除全局变量定义，改为extern声明
extern DisplayMode currentMode;
extern SystemStatus currentStatus;

// 自定义字符定义 - 简化版本
byte batteryChar[] = {
    B01110,
    B11011,
    B10001,
    B10001,
    B10001,
    B10001,
    B10001,
    B11111
};

byte alertChar[] = {
    B00100,
    B01110,
    B11111,
    B11111,
    B11111,
    B01110,
    B00100,
    B00000
};

byte robotChar[] = {
    B01110,
    B11111,
    B10101,
    B11111,
    B01110,
    B00100,
    B00100,
    B00000
};

// 初始化LCD
void initLCD() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    // 创建自定义字符
    createCustomChar(0, batteryChar);
    createCustomChar(1, alertChar);
    createCustomChar(2, robotChar);
    // 显示启动信息
    printText(0, 0, "RoboGuard");
    printText(0, 1, "Starting...");
    delay(2000);
    // showStatus(STATUS_IDLE); // 避免初始化时依赖全局变量，防乱码
}

// 基础显示函数
void clearDisplay() {
    lcd.clear();
}

void setCursor(int col, int row) {
    lcd.setCursor(col, row);
}

void printText(const char* text) {
    lcd.print(text);
}

void printText(int col, int row, const char* text) {
    lcd.setCursor(col, row);
    lcd.print(text);
}

// 状态显示 - 针对16x2优化
void showStatus(SystemStatus status) {
    clearDisplay();
    printText(0, 0, "Status:");
    printText(0, 1, formatStatus(status).c_str());
    currentStatus = status;
}

void showStatusWithSpeed(SystemStatus status, int speed) {
    clearDisplay();
    printText(0, 0, formatStatus(status).c_str());
    printText(0, 1, "Speed:");
    printText(7, 1, formatSpeed(speed).c_str());
    currentStatus = status;
}

void showStatusWithDistance(SystemStatus status, int distance) {
    clearDisplay();
    printText(0, 0, formatStatus(status).c_str());
    printText(0, 1, "Dist:");
    printText(6, 1, formatDistance(distance).c_str());
    currentStatus = status;
}

// 距离显示 - 一行显示三个距离
void showDistance(int distL, int distM, int distR) {
    clearDisplay();
    printText(0, 0, "Distance:");
    printText(0, 1, "L:");
    printText(2, 1, String(distL).c_str());
    printText(5, 1, "M:");
    printText(7, 1, String(distM).c_str());
    printText(10, 1, "R:");
    printText(12, 1, String(distR).c_str());
}

void showDistanceSimple(int distance) {
    clearDisplay();
    printText(0, 0, "Distance:");
    printText(0, 1, formatDistance(distance).c_str());
}

// 速度显示 - 一行显示目标速度，一行显示实际速度
void showSpeed(int targetSpeed, int actualSpeed) {
    clearDisplay();
    printText(0, 0, "Target:");
    printText(7, 0, formatSpeed(targetSpeed).c_str());
    printText(0, 1, "Actual:");
    printText(7, 1, formatSpeed(actualSpeed).c_str());
}

void showSpeedAll(int speedLF, int speedRF, int speedLB, int speedRB) {
    clearDisplay();
    printText(0, 0, "Speed:");
    printText(0, 1, "LF:");
    printText(3, 1, String(speedLF).c_str());
    printText(6, 1, "RF:");
    printText(9, 1, String(speedRF).c_str());
    printText(12, 1, "LB:");
    printText(15, 1, String(speedLB).c_str());
}

// 警报显示 - 突出显示详细报告
void showAlert(const char* alert) {
    clearDisplay();
    showCustomChar(1, 0, 0); // 警报图标
    printText(2, 0, "ALERT!");
    printText(0, 1, alert); // 详细报告信息
}

void showAlertWithCode(const char* alert, int code) {
    clearDisplay();
    showCustomChar(1, 0, 0);
    printText(2, 0, "ALERT!");
    printText(0, 1, alert);
    printText(12, 1, "E");
    printText(13, 1, String(code).c_str());
}

// 调试显示 - 显示关键调试信息
void showDebug(const char* info, int value) {
    clearDisplay();
    printText(0, 0, info);
    printText(0, 1, String(value).c_str());
    currentMode = MODE_DEBUG;
}

void showDebugTwoValues(int val1, int val2) {
    clearDisplay();
    printText(0, 0, "Debug:");
    printText(0, 1, String(val1).c_str());
    printText(8, 1, String(val2).c_str());
}

// 滚动显示 - 长文本滚动
void scrollText(const char* text, int row) {
    int len = strlen(text);
    if (len <= 16) {
        printText(0, row, text);
        return;
    }
    for (int i = 0; i <= len - 16; i++) {
        clearDisplay();
        char temp[17];
        strncpy(temp, text + i, 16);
        temp[16] = '\0';
        printText(0, row, temp);
        delay(300);
    }
}

void scrollTextSimple(const char* text) {
    scrollText(text, 0);
}

// 自定义字符显示
void createCustomChar(byte location, byte charmap[]) {
    lcd.createChar(location, charmap);
}

void showCustomChar(byte location, int col, int row) {
    lcd.setCursor(col, row);
    lcd.write(location);
}

// 动画效果 - 简单的动画
void showLoading() {
    const char* frames[] = {"|", "/", "-", "\\"};
    for (int i = 0; i < 4; i++) {
        printText(15, 0, frames[i]);
        delay(200);
    }
}

void showPatrol() {
    const char* frames[] = {"->", "->", "->", "->"};
    for (int i = 0; i < 4; i++) {
        clearDisplay();
        printText(0, 0, "Patrolling");
        printText(i * 4, 1, frames[i]);
        delay(300);
    }
}

void showAlertBlink() {
    for (int i = 0; i < 3; i++) {
        clearDisplay();
        printText(0, 0, "ALERT!");
        delay(500);
        clearDisplay();
        delay(200);
    }
}

// 组合显示 - 常用组合
// 主界面显示 - 第二行显示当前状态
void showMainScreen(SystemStatus status, int speed) {
    clearDisplay();
    printText(0, 0, formatStatus(status).c_str());
    printText(8, 0, "S:");
    printText(10, 0, formatSpeed(speed).c_str());
    printText(0, 1, getStatusText(status));
}

void showSensorScreen(int distL, int distM, int distR, bool tracking) {
    clearDisplay();
    printText(0, 0, "Sensors:");
    printText(0, 1, "L:");
    printText(2, 1, String(distL).c_str());
    printText(5, 1, "M:");
    printText(7, 1, String(distM).c_str());
    printText(10, 1, "R:");
    printText(12, 1, String(distR).c_str());
    if (tracking) {
        printText(15, 1, "T");
    }
}

// 格式化函数 - 针对16字符限制
String formatDistance(int distance) {
    if (distance > 999) {
        return String(distance / 1000) + "m";
    } else {
        return String(distance) + "cm";
    }
}

String formatSpeed(int speed) {
    return String(speed) + "/255";
}

String formatStatus(SystemStatus status) {
    switch (status) {
        case STATUS_IDLE: return "IDLE";
        case STATUS_PATROL: return "PATROL";
        case STATUS_TRACKING: return "TRACK";
        case STATUS_AVOIDING: return "AVOID";
        case STATUS_ALERT: return "ALERT!";
        case STATUS_ERROR: return "ERROR!";
        default: return "UNKNOWN";
    }
}

// 新增：根据SystemStatus返回友好状态文本
const char* getStatusText(SystemStatus status) {
    switch (status) {
        case STATUS_IDLE: return "待机";
        case STATUS_PATROL: return "正常巡逻";
        case STATUS_TRACKING: return "跟踪";
        case STATUS_AVOIDING: return "避障";
        case STATUS_ALERT: return "警告";
        case STATUS_ERROR: return "错误";
        default: return "未知";
    }
} 