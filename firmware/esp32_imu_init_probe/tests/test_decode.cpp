#include "decode.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
    assert(signed12(0xF1, 0x7F) == 2047);
    assert(signed12(1, 0x80) == -2048);
    assert(signed12(0xF1, 0xFF) == -1);
    assert(signed16(0, 0x80) == -32768);
    assert(signed16(255, 127) == 32767);
    assert(signed16(255, 255) == -1);
    assert(accelScale(5) == 4);
    assert(accelScale(0) == 0);
    assert(gyroScale(2) == 500);
    assert(gyroScale(7) == 0);
    assert(accelReady(0x10) && !accelReady(0));
    assert(gyroBusy(8) && !gyroBusy(0));
    assert(std::abs(512 * accelScale(5) * 9.81 / 2048 - 9.81) < 1e-6);
    std::cout << "PASS signed decoding, scales, calibration status\n";
}
