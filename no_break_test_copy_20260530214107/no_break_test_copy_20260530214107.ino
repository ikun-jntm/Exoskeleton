#include <Arduino.h>
#include <Wire.h>
#include <TM1650.h>        
#include "UnitreeGoMotor.h"
#include "ExoController.h" 

// ================= 硬件引脚配置 =================
#define RX_PIN 25
#define TX_PIN 26
#define EN_PIN 27
#define BAUDRATE 4000000

// UI 硬件引脚
#define POT_PIN 2       // 旋转电位器
// 【新增】按键与 LED 引脚
#define BTN_PIN 39      // 模式切换按钮
#define LED_PIN 5      // 设置模式指示灯

// ================= 实例化对象 =================
HardwareSerial MotorSerial(1);
UnitreeGoMotor motor_left(&MotorSerial, 1, EN_PIN);
UnitreeGoMotor motor_right(&MotorSerial, 0, EN_PIN);
ExoController exo_left(&motor_left, "Left", 1);
ExoController exo_right(&motor_right, "Right", -1);
TM1650 display; 

// ================= 全局 UI 变量 =================
unsigned long lastUITime = 0;
float filtered_pot_val = 0.0; 

// 【新增】按键消抖与状态机变量
int buttonState = HIGH;             // 按钮当前稳定状态
int lastButtonReading = LOW;       // 按钮上一次的物理读数
unsigned long lastDebounceTime = 0; // 上次抖动时间
const unsigned long debounceDelay = 50; // 消抖延迟 (50毫秒)

bool isSettingMode = false;         // 系统是否处于设置模式
//旋转编码器
const int pinA = 33;
const int pinB = 32;
int counter = 0; 
int lastStateA;  
// ================= 主程序 =================
void setup() {
    Serial.begin(115200); 
    //拉起旋转编码器针口
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    
    // 初始化 UI 硬件
    pinMode(POT_PIN, INPUT);
    // ESP32-S3 的 39 号针脚默认没上拉电阻，这里用 INPUT_PULLUP。
    // 如果按钮按下是接 GND，这行代码能保证松开时是 HIGH。
    pinMode(BTN_PIN, INPUT_PULLUP); 
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // 开机默认运行模式，灯灭
    
    Wire.begin(); 
    display.init();                       
    display.displayOn();
    display.setBrightness(3);             
    
    // 初始化电机硬件
    MotorSerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
    motor_left.begin();
    motor_right.begin();
    
    Serial.println("【系统】双电机初始化中...1");
    delay(2000); 

    exo_left.begin();
    exo_left.calibrateZeroPosition();
    exo_right.begin();
    exo_right.calibrateZeroPosition();
    
    // 开机时在数码管上显示初始扭矩 0.00
    display.displayString("0.00");
    lastStateA = digitalRead(pinA);   
}

void loop() {
    // ==========================================================
    // 任务 1：按键监听与状态切换 (非阻塞消抖算法) - 保持不变
    // ==========================================================
    int currentReading = digitalRead(BTN_PIN);

    if (currentReading != lastButtonReading) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (currentReading != buttonState) {
            buttonState = currentReading;
            if (buttonState == HIGH) {
                isSettingMode = !isSettingMode; // 翻转模式
                digitalWrite(LED_PIN, isSettingMode ? HIGH : LOW);
                exo_left.setSettingMode(isSettingMode);
                exo_right.setSettingMode(isSettingMode);
                
                if (isSettingMode) {
                    Serial.println(">>> 切换至【设置模式】：动力切断，可旋转编码器调节扭矩 <<<");
                } else {
                    Serial.println(">>> 切换至【运行模式】：编码器已锁定，动力恢复！ <<<");
                }
            }
        }
    }
    lastButtonReading = currentReading; 

    // ==========================================================
    // 【修正】任务 1.5：编码器高频轮询（移出UI定时器，全速运行）
    // ==========================================================
    if (isSettingMode) {
        int currentStateA = digitalRead(pinA);
        
        // 只有当 A 相状态发生改变时才处理
        if (currentStateA != lastStateA) {     
            // 如果 A 相变化后的状态与 B 相当前状态不同，说明是顺时针
            if (digitalRead(pinB) != currentStateA) { 
                counter++; 
            } else {
                counter--; 
            }
            
            // 【关键修正 1】：限制 counter 的范围，防止无限增大或变成负数
            // 假设我们希望扭矩 0.00 ~ 0.80，每拨动一格改变 0.01，那么 counter 范围设为 0~80
            if (counter < 0)  counter = 0;
            if (counter > 80) counter = 80;
        }  
        lastStateA = currentStateA; // 【关键修正 2】：必须更新状态
    }

    // ==========================================================
    // 任务 2：电机极速控制循环 (500Hz) - 保持不变
    // ==========================================================
    exo_left.run();
    exo_right.run();

    // ==========================================================
    // 任务 3：UI 刷新与数据显示 (10Hz)
    // ==========================================================
    unsigned long currentMillis = millis();
    if (currentMillis - lastUITime >= 100) {
        lastUITime = currentMillis;

        if (isSettingMode) {
            // 【关键修正 3】：重新设计编码器数值到扭矩的转换逻辑
            // 一格代表 0.01 N·m
            float current_max_torque = counter * 0.01; 

            if (current_max_torque < 0.01) current_max_torque = 0.0;
            if (current_max_torque > 0.79) current_max_torque = 0.80;

            // 实时下发最大扭矩限制
            exo_left.setMaxAssistTorque(current_max_torque);
            exo_right.setMaxAssistTorque(current_max_torque);

            // 实时刷新数码管
            char display_buf[5];
            sprintf(display_buf, "%.2f", current_max_torque);
            display.displayString(display_buf);
        }
    }
}