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
UnitreeGoMotor motor_left(&MotorSerial, 0, EN_PIN);
UnitreeGoMotor motor_right(&MotorSerial, 1, EN_PIN);
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

// ================= 主程序 =================
void setup() {
    Serial.begin(115200); 
    
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
    
    Serial.println("【系统】双电机初始化中...");
    delay(2000); 

    exo_left.begin();
    exo_left.calibrateZeroPosition();
    exo_right.begin();
    exo_right.calibrateZeroPosition();
    
    // 开机时在数码管上显示初始扭矩 0.00
    display.displayString("0.00");
}

void loop() {
    // ==========================================================
    // 任务 1：按键监听与状态切换 (非阻塞消抖算法)
    // ==========================================================
    int currentReading = digitalRead(BTN_PIN);

    // 如果引脚电平发生跳变（不论是按下还是松开），重置消抖计时器
    if (currentReading != lastButtonReading) {
        lastDebounceTime = millis();
    }

    // 如果引脚电平保持稳定的时间超过了 50ms (说明不是干扰)
    if ((millis() - lastDebounceTime) > debounceDelay) {
        
        // 确认状态确实发生了改变
        if (currentReading != buttonState) {
            buttonState = currentReading;

            // 【触发条件】：按下后松开 (电平由 LOW 变回 HIGH)
            if (buttonState == HIGH) {
                isSettingMode = !isSettingMode; // 翻转模式
                
                // 1. 切换 LED 状态
                digitalWrite(LED_PIN, isSettingMode ? HIGH : LOW);
                
                // 2. 通知下层电机控制器
                exo_left.setSettingMode(isSettingMode);
                exo_right.setSettingMode(isSettingMode);
                
                // 3. 串口提示
                if (isSettingMode) {
                    Serial.println(">>> 切换至【设置模式】：动力切断，可旋转电位器调节扭矩 <<<");
                } else {
                    Serial.println(">>> 切换至【运行模式】：电位器已锁定，动力恢复！ <<<");
                }
            }
        }
    }
    lastButtonReading = currentReading; // 保存本次物理读数

    // ==========================================================
    // 任务 2：电机极速控制循环 (500Hz)
    // ==========================================================
    exo_left.run();
    exo_right.run();

    // ==========================================================
    // 任务 3：UI 刷新与电位器读取 (10Hz)
    // ==========================================================
    unsigned long currentMillis = millis();
    if (currentMillis - lastUITime >= 100) {
        lastUITime = currentMillis;

        // 【防误触安全锁】：只有处于设置模式时，才会读取电位器并更新扭矩
        if (isSettingMode) {
            int raw_pot = analogRead(POT_PIN);
            filtered_pot_val = 0.8 * filtered_pot_val + 0.2 * raw_pot;
            
            float current_max_torque = (filtered_pot_val / 4095.0) * 0.80;
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
        // 如果不在设置模式（运行模式中），完全跳过电位器读取，维持上一次设置的值，数码管也保持常亮。
    }
}