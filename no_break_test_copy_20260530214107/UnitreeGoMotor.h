#ifndef UNITREE_GO_MOTOR_H
#define UNITREE_GO_MOTOR_H

#include <Arduino.h>

#pragma pack(1)
// 发送协议 (17 Bytes)
typedef struct {
    uint8_t  head[2];
    uint8_t  id_mode;     // ID[0:3] Mode[4:7]
    int16_t  T;
    int16_t  W;
    int32_t  Pos;
    int16_t  Kp;
    int16_t  Kw;
    uint16_t CRC;
} CmdPacket;

// 接收协议 (16 Bytes)
typedef struct {
    uint8_t  head[2];
    uint8_t  id_mode;     // ID[0:3] Mode[4:7]
    int16_t  T;
    int16_t  W;
    int32_t  Pos;
    int8_t   Temp;
    uint8_t  MError;
    int16_t  Force;
    uint16_t CRC;
} ResPacket;
#pragma pack()

class UnitreeGoMotor {
public:
    // --- 新增：curr_ID ---
    int   curr_ID;      // 当前反馈回来的电机ID
    float curr_T;
    float curr_W;
    float curr_Pos;
    int   curr_Temp;
    int   curr_Error;
    bool  is_connected;

    UnitreeGoMotor(HardwareSerial* serial, uint8_t id, int en_pin);
    void begin();
    void control(uint8_t mode, float T, float W, float Pos, float Kp, float Kw);
    bool receive();

private:
    HardwareSerial* _serial;
    uint8_t _id;
    int _en_pin;
    uint16_t calcCRC(const uint8_t *data, uint16_t len);
};

#endif