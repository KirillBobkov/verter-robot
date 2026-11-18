#include "imu_checked.h"
#include <cassert>
#include <map>
#include <iostream>
struct Bus {
    uint32_t clock = 0;
    int writes = 0, reads = 0;
    bool failRead = false, failWrite = false, timeout = false;
    std::map<int, uint8_t> regs;
    bool read(uint8_t a, uint8_t r, uint8_t &v) {
        ++reads;
        if (failRead)
            return false;
        if (r == 0x36 && a == 0x19) {
            v = timeout ? 0 : 0x10;
            return true;
        }
        if (r == 0x32 && a == 0x69) {
            v = timeout ? 8 : 0;
            return true;
        }
        v = regs[a * 256 + r];
        return true;
    }
    bool write(uint8_t, uint8_t, uint8_t) {
        ++writes;
        return !failWrite;
    }
    uint32_t milliseconds() { return clock; }
    void sleep(uint32_t ms) { clock += ms; }
};
int main() {
    Bus b;
    assert(imu_checked::calibrate(b));
    assert(b.writes == 6);
    Bus w;
    w.failWrite = true;
    assert(!imu_checked::calibrate(w));
    assert(w.writes == 1);
    Bus r;
    r.failRead = true;
    assert(!imu_checked::calibrate(r));
    assert(r.writes == 3);
    Bus t;
    t.timeout = true;
    t.clock = 0xFFFFFF00;
    uint32_t start = t.clock;
    assert(!imu_checked::calibrate(t));
    assert(uint32_t(t.clock - start) == 1500);
    assert(t.writes == 3);
    Bus g;
    g.timeout = true;
    assert(!imu_checked::waitBit(g, 0x69, 0x32, 8, false, 5000));
    assert(g.clock == 5000);
    Bus c;
    for (auto v : {std::initializer_list<int>{0x19, 0, 0xFA},
                   {0x69, 0, 0x0F},
                   {0x19, 15, 5},
                   {0x69, 15, 2},
                   {0x19, 16, 12},
                   {0x69, 16, 0x84}}) {
        auto i = v.begin();
        int a = *i++, reg = *i++, val = *i;
        c.regs[a * 256 + reg] = val;
    }
    assert(imu_checked::config(c));
    c.regs[0x19 * 256 + 15] = 3;
    assert(!imu_checked::config(c));
    c.regs[0x19 * 256 + 15] = 5;
    c.failRead = true;
    assert(!imu_checked::config(c));
    std::cout << "PASS calibration success/write failure/read failure/timeouts/clock wrap/config readback\n";
}
