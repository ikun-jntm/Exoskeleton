#ifndef EXO_CONTROLLER_H
#define EXO_CONTROLLER_H

#include <Arduino.h>
#include "UnitreeGoMotor.h"

// ==========================================================
// 1. 数据结构与状态定义
// ==========================================================
enum ExoState {
    STATE_STANDBY, // 待机相
    STATE_SWING,   // 摆动相
    STATE_STANCE   // 支撑相
};

struct MotorData {
    float angle;        
    float velocity;     
    float velocity_f;   
    float acceleration; 
    float torque;       
};

// ==========================================================
// 2. 外骨骼控制器类声明
// ==========================================================
class ExoController {
public:
    // 构造函数：包含电机指针、名字、安装方向乘数
    ExoController(UnitreeGoMotor* motor_ptr, String exo_name, int direction_multiplier);

    // 初始化与运行函数
    void begin();
    void calibrateZeroPosition();
    void run(); 

    // 设置动态扭矩上限
    void setMaxAssistTorque(float max_torque);
    // 【新增】切换设置模式接口
    void setSettingMode(bool active);

private:
    UnitreeGoMotor* motor; 
    String name;              // 左右腿名称
    int dir_mult;             // 安装方向乘数 (1 或 -1)
    float max_assist_torque;  // 动态扭矩上限
    // 【新增】设置模式标志位
    bool setting_mode_active;

    // 内部状态与数据
    ExoState currentState;
    MotorData motor_data;
    
    unsigned long lastLoopTime;
    unsigned long lowSpeedStartTime;
    float zero_pos_offset;

    // 私有核心函数
    void executeControlStrategy();
    void readAndProcessMotorData(float dt);
    void updateStateMachine();
    void printCurrentState();

    // ==========================================================
    // 3. 核心调参区 (可根据实际情况修改)
    // ==========================================================
    // 物理参数
    static constexpr float GEAR_RATIO = 6.33;             // 电机减速比
    static constexpr unsigned long LOOP_PERIOD_US = 2000; // 500Hz 控制周期

    // 安全软限位保护
    static constexpr float LIMIT_ANGLE_MAX = 70.0;     // 极限前抬角度
    static constexpr float LIMIT_ANGLE_MIN = -30.0;    // 极限后伸角度
    static constexpr float LIMIT_VELOCITY_MAX = 330.0; // 极限角速度

    // 状态机相变阈值
    static constexpr float THRESHOLD_ANGLE_SWING = 30.0;  
    static constexpr float THRESHOLD_VEL_SWING = 60.0;    
    static constexpr float THRESHOLD_ANGLE_STANCE = -20.0;
    static constexpr float THRESHOLD_VEL_STANCE = -60.0;  
    static constexpr float THRESHOLD_VEL_STANDBY = 30.0;  

    
    static constexpr unsigned long STANDBY_TIME_MS = 500; 

    // 意图阻断助力参数
    static constexpr float ASSIST_KV = 0.005;       // 速度助力增益
    static constexpr float CONSTANT_KW = 0.01;       // 基础阻尼
};

#endif // EXO_CONTROLLER_H