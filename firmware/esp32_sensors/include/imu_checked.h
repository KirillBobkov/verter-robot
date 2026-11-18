#pragma once
#include <stdint.h>
#include <initializer_list>
// Bus implements read/write and milliseconds/sleep. Shared with host-side fault tests.
namespace imu_checked {
template <class Bus> bool expect(Bus &b, uint8_t a, uint8_t reg, uint8_t value, uint8_t mask = 255) {
    uint8_t v = 0;
    return b.read(a, reg, v) && ((v & mask) == value);
}
template <class Bus> bool waitBit(Bus &b, uint8_t a, uint8_t reg, uint8_t mask, bool set, uint32_t timeout) {
    uint32_t start = b.milliseconds();
    do {
        uint8_t v;
        if (!b.read(a, reg, v))
            return false;
        if (bool(v & mask) == set)
            return true;
        b.sleep(10);
    } while (uint32_t(b.milliseconds() - start) < timeout);
    return false;
}
template <class Bus> bool config(Bus &b) {
    return expect(b, 0x19, 0, 0xFA) && expect(b, 0x69, 0, 0x0F) && expect(b, 0x19, 15, 5, 15) &&
           expect(b, 0x69, 15, 2, 7) && expect(b, 0x19, 16, 12, 31) && expect(b, 0x69, 16, 4, 7) &&
           expect(b, 0x19, 17, 0, 0xE0) && expect(b, 0x69, 17, 0, 0xA0) && expect(b, 0x19, 19, 0, 0xC0) &&
           expect(b, 0x69, 19, 0, 0xC0);
}
template <class Bus> bool calibrate(Bus &b) {
    if (!b.write(0x19, 0x36, 0x80) || !b.write(0x19, 0x37, 0x20))
        return false;
    for (uint8_t v : {0x20, 0x40, 0x60})
        if (!b.write(0x19, 0x36, v) || !waitBit(b, 0x19, 0x36, 0x10, true, 1500))
            return false;
    return b.write(0x69, 0x32, 0xFF) && waitBit(b, 0x69, 0x32, 8, false, 5000);
}
} // namespace imu_checked
