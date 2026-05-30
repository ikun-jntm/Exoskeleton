#include "UnitreeGoMotor.h"

UnitreeGoMotor::UnitreeGoMotor(HardwareSerial* serial, uint8_t id, int en_pin) {
    _serial = serial;
    _id = id;
    _en_pin = en_pin;
    is_connected = false;
    curr_ID = -1; // 初始化为无效ID
}

void UnitreeGoMotor::begin() {
    pinMode(_en_pin, OUTPUT);
    digitalWrite(_en_pin, LOW);
}

uint16_t UnitreeGoMotor::calcCRC(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0x0000;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0x8408;
            else crc >>= 1;
        }
    }
    return crc;
}

void UnitreeGoMotor::control(uint8_t mode, float T, float W, float Pos, float Kp, float Kw) {
    CmdPacket cmd;
    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEE;
    cmd.id_mode = (_id & 0x0F) | ((mode & 0x07) << 4);

    cmd.T   = (int16_t)(T * 256.0f);
    cmd.W   = (int16_t)(W / (2.0f * PI) * 256.0f);
    cmd.Pos = (int32_t)(Pos / (2.0f * PI) * 32768.0f);
    cmd.Kp  = (int16_t)(Kp * 1280.0f);
    cmd.Kw  = (int16_t)(Kw * 1280.0f);

    cmd.CRC = calcCRC((uint8_t*)&cmd, 15);

    digitalWrite(_en_pin, HIGH);
    _serial->write((uint8_t*)&cmd, sizeof(cmd));
    _serial->flush();
    digitalWrite(_en_pin, LOW);
}

bool UnitreeGoMotor::receive() {
    if (_serial->available() < 16) return false;

    if (_serial->peek() != 0xFD) {
        _serial->read();
        return false;
    }

    uint8_t buf[16];
    _serial->readBytes(buf, 16);

    if (buf[1] != 0xEE) return false;

    uint16_t recv_crc;
    memcpy(&recv_crc, &buf[14], 2);
    if (calcCRC(buf, 14) != recv_crc) return false;

    ResPacket* res = (ResPacket*)buf;
    
    // --- 新增：解析ID ---
    // 0x0F 是二进制 00001111，用于保留低4位（ID），过滤掉高4位（Mode）
    curr_ID = res->id_mode & 0x0F; 

    curr_T   = res->T / 256.0f;
    curr_W   = res->W / 256.0f * 2.0f * PI;
    curr_Pos = res->Pos / 32768.0f * 2.0f * PI;
    curr_Temp = res->Temp;
    curr_Error = res->MError & 0x07;
    
    is_connected = true;
    return true;
}