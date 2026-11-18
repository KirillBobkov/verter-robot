#pragma once
#include <stdint.h>
inline int signed16(uint8_t lo, uint8_t hi) {
    unsigned v = unsigned(lo) | (unsigned(hi) << 8);
    return v >= 32768 ? int(v) - 65536 : int(v);
}
inline int signed12(uint8_t lo, uint8_t hi) {
    unsigned v = (unsigned(lo) | (unsigned(hi) << 8)) >> 4;
    return v >= 2048 ? int(v) - 4096 : int(v);
}
inline float accelScale(uint8_t r) {
    switch (r & 15) {
    case 3:
        return 2;
    case 5:
        return 4;
    case 8:
        return 8;
    case 12:
        return 16;
    default:
        return 0;
    }
}
inline float gyroScale(uint8_t r) {
    switch (r & 7) {
    case 0:
        return 2000;
    case 1:
        return 1000;
    case 2:
        return 500;
    case 3:
        return 250;
    case 4:
        return 125;
    default:
        return 0;
    }
}
inline bool accelReady(uint8_t r) {
    return (r & 0x10) != 0;
}
inline bool gyroBusy(uint8_t r) {
    return (r & 8) != 0;
}
