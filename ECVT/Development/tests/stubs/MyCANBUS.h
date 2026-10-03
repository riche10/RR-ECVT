#pragma once
#include <cstdint>
struct CAN_Data { uint16_t id = 0; uint8_t x[8] = {0}; };
inline CAN_Data MOTEC_Data_RX0, TCU_Data_TX, mockLastCan;
inline unsigned mockCanFrames = 0;
inline constexpr uint16_t TCU_id0 = 0x130;
struct MockCan { void events() {} };
inline MockCan Can2;
inline void MyCanStart(bool) {}
inline bool MyCanSend(CAN_Data data) {
    mockLastCan = data;
    ++mockCanFrames;
    return true;
}
