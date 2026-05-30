#include "ExoController.h"

// 构造函数
ExoController::ExoController(UnitreeGoMotor* motor_ptr, String exo_name, int direction_multiplier) {
    motor = motor_ptr;
    name = exo_name;
    dir_mult = direction_multiplier;
    currentState = STATE_STANDBY;
    
    max_assist_torque = 0.0; // 默认开机扭矩上限为0，等待电位器设置
    setting_mode_active = false;
    
    lastLoopTime = 0;
    lowSpeedStartTime = 0;
    zero_pos_offset = 0.0;
    motor_data = {0.0, 0.0, 0.0, 0.0, 0.0};
}

// 初始化时间
void ExoController::begin() {
    lastLoopTime = micros();
}
// 【新增】动态设置模式
void ExoController::setSettingMode(bool active) {
    setting_mode_active = active;
    if (active) {
        // 安全机制：一旦进入设置模式，强制重置状态机为待机，防止切回时出现突变
        currentState = STATE_STANDBY; 
    }
}
// 零点校准
void ExoController::calibrateZeroPosition() {
    Serial.printf("[%s] 正在进行零点校准，请保持垂直静止...\n", name.c_str());
    motor->control(1, 0, 0, 0, 0, 0); 
    delay(10); 
    
    if (motor->receive()) {
        zero_pos_offset = motor->curr_Pos;
        Serial.printf("[%s] 成功: 转子零点已校准: %.2f rad\n", name.c_str(), zero_pos_offset);
    } else {
        Serial.printf("[%s] 警告: 未连接到电机，零点校准失败！\n", name.c_str());
    }
}

// 动态设置扭矩上限
void ExoController::setMaxAssistTorque(float max_torque) {
    max_assist_torque = max_torque;
}

// 主运行循环
void ExoController::run() {
    unsigned long currentTime = micros();
    
    if (currentTime - lastLoopTime >= LOOP_PERIOD_US) {
        float dt = (currentTime - lastLoopTime) / 1000000.0;
        lastLoopTime = currentTime;
        
        executeControlStrategy();
        readAndProcessMotorData(dt);
        updateStateMachine();
        printCurrentState();
    }
}

// 1. 执行控制策略 (仅速度跟随版 - 不带加速度判断)
void ExoController::executeControlStrategy() {
    // 如果是设置模式，保持 FOC 0 力矩
    if (setting_mode_active) {
        motor->control(1, 0.0, 0, 0, 0, 0.0);
        return;
    }

    float raw_target_torque = 0.0; 
    float damping_kw = CONSTANT_KW; // 使用基础阻尼

    float w = motor_data.velocity_f;

    switch (currentState) {
        case STATE_STANDBY:
            raw_target_torque = 0.0;
            break;

        case STATE_SWING:
            // 摆动相：只要速度 w 是正的，就按比例助力
            if (w > 0) {
                raw_target_torque = ASSIST_KV * w;
            } else {
                // 发生过冲或回摆时，强制零力矩
                raw_target_torque = 0.0;
            }
            break;

        case STATE_STANCE:
            // 支撑相：只要速度 w 是负的，就按比例助力
            if (w < 0) {
                raw_target_torque = ASSIST_KV * w;
            } else {
                // 发生过冲或回弹时，强制零力矩
                raw_target_torque = 0.0;
            }
            break;
    }

    // 力矩输出低通滤波器 (依然建议保留，能显著提升手感)
    static float smoothed_torque = 0.0;
    smoothed_torque = 0.4 * smoothed_torque + 0.6 * raw_target_torque;

    // 绝对安全限幅 (使用电位器传进来的动态变量)
    if (smoothed_torque > max_assist_torque) smoothed_torque = max_assist_torque;
    if (smoothed_torque < -max_assist_torque) smoothed_torque = -max_assist_torque;

    // 发给底层硬件前，按安装方向镜像翻转
    float hardware_torque = smoothed_torque * dir_mult;

    // 发送指令
    motor->control(1, hardware_torque, 0, 0, 0, damping_kw);
}

// 2 & 3. 读取数据与预处理 (带镜像读取)
void ExoController::readAndProcessMotorData(float dt) {
    unsigned long wait_start = micros();
    bool success = false;
    
    while (micros() - wait_start < 1000) {
        if (motor->receive()) {
            success = true;
            break;
        }
    }

    if (success && dt > 0.0001 && dt < 0.1) { 
        // 原始数据乘以 dir_mult 进行坐标镜像
        motor_data.angle = (((motor->curr_Pos - zero_pos_offset) / GEAR_RATIO) * (180.0 / PI)) * dir_mult;
        motor_data.velocity = ((motor->curr_W / GEAR_RATIO) * (180.0 / PI)) * dir_mult;
        motor_data.torque = (motor->curr_T * GEAR_RATIO) * dir_mult;

        // 静止死区消噪
        if (abs(motor_data.velocity) < 2.0) {
            motor_data.velocity = 0.0;
        }

        // 速度低通滤波
        float last_vel_f = motor_data.velocity_f; 
        motor_data.velocity_f = 0.95 * last_vel_f + 0.05 * motor_data.velocity;

        // 角加速度二次低通滤波
        float raw_accel = (motor_data.velocity_f - last_vel_f) / dt;
        motor_data.acceleration = 0.9 * motor_data.acceleration + 0.1 * raw_accel;
    }
}

// 4. 状态机判定
void ExoController::updateStateMachine() {
    if (motor_data.angle >= LIMIT_ANGLE_MAX || 
        motor_data.angle <= LIMIT_ANGLE_MIN || 
        abs(motor_data.velocity_f) >= LIMIT_VELOCITY_MAX) {
        
        currentState = STATE_STANDBY; 
        lowSpeedStartTime = 0; 
        return; 
    }

    if (abs(motor_data.velocity_f) < THRESHOLD_VEL_STANDBY) {
        if (lowSpeedStartTime == 0) {
            lowSpeedStartTime = millis();
        } else if (millis() - lowSpeedStartTime > STANDBY_TIME_MS) {
            currentState = STATE_STANDBY;
        }
    } else {
        lowSpeedStartTime = 0; 
    }

    switch (currentState) {
        case STATE_STANDBY:
            if (motor_data.angle < THRESHOLD_ANGLE_SWING && motor_data.velocity_f > THRESHOLD_VEL_SWING) {
                currentState = STATE_SWING;
            } else if (motor_data.angle > THRESHOLD_ANGLE_STANCE && motor_data.velocity_f < THRESHOLD_VEL_STANCE) {
                currentState = STATE_STANCE;
            }
            break;
        case STATE_SWING:
            if (motor_data.angle > THRESHOLD_ANGLE_STANCE && motor_data.velocity_f < THRESHOLD_VEL_STANCE) {
                currentState = STATE_STANCE;
            }
            break;
        case STATE_STANCE:
            if (motor_data.angle < THRESHOLD_ANGLE_SWING && motor_data.velocity_f > THRESHOLD_VEL_SWING) {
                currentState = STATE_SWING;
            }
            break;
    }
}

// 5. 打印状态
void ExoController::printCurrentState() {
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 50) {
        lastPrintTime = millis();
        
        String stateStr = "";
        switch (currentState) {
            case STATE_STANDBY: stateStr = "STANDBY"; break;
            case STATE_SWING:   stateStr = "SWING"; break;
            case STATE_STANCE:  stateStr = "STANCE"; break;
        }

        Serial.printf("[%s] State:%s | Ang:%.1f | Vel:%.1f | Acc:%.1f | Trq:%.2f | MaxT:%.2f\n", 
                      name.c_str(), 
                      stateStr.c_str(), 
                      motor_data.angle, 
                      motor_data.velocity_f, 
                      motor_data.acceleration,
                      motor_data.torque,
                      max_assist_torque); // 顺便把当前的扭矩上限打印出来方便确认
    }
}