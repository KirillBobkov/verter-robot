#include "lslidar_driver/m10p_scan.h"
#include <iostream>
#include <string>

using namespace lslidar_driver;

static void check(bool condition, const std::string &message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

static std::vector<uint8_t> packet(int degree, size_t count, int pad = -1, uint16_t value = 2000) {
    const size_t slots = count + (pad >= 0 ? 1 : 0);
    std::vector<uint8_t> bytes(20 + slots * 2, 0);
    bytes[0] = 0xa5;
    bytes[1] = 0x5a;
    bytes[2] = bytes.size() >> 8;
    bytes[3] = bytes.size() & 255;
    bytes[4] = (degree * 100) >> 8;
    bytes[5] = (degree * 100) & 255;
    bytes[bytes.size() - 2] = 0xfa;
    bytes[bytes.size() - 1] = 0xfb;
    for (size_t i = 0; i < slots; ++i) {
        const auto raw = static_cast<int>(i) == pad ? 0xffff : value;
        bytes[8 + 2 * i] = raw >> 8;
        bytes[9 + 2 * i] = raw & 255;
    }
    return bytes;
}

static void decode_tests() {
    const auto bytes = packet(345, 5, 2);
    const auto points = decode_m10p(bytes.data(), bytes.size(), false);
    check(points.size() == 5, "FFFF padding must be compacted before angle interpolation");
    for (size_t i = 0; i < points.size(); ++i) {
        check(std::abs(points[i].degree - (345 + 3 * i)) < 1e-6, "padding shifted physical angles");
    }
    auto reflected = packet(0, 5, -1, 0x83e8);
    const auto high = decode_m10p(reflected.data(), reflected.size(), true);
    check(high[0].range == 1.0f && high[0].intensity == 255, "high reflection decode");
    auto damaged = bytes;
    damaged.back() = 0;
    bool rejected = false;
    try {
        decode_m10p(damaged.data(), damaged.size(), false);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    check(rejected, "invalid footer accepted");
    rejected = false;
    try {
        decode_m10p(nullptr, 0, false);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    check(rejected, "truncated packet accepted");
}

static void sweep_tests() {
    M10pScanAssembler assembler(1800, .1f, 25.0f);
    std::vector<M10pSweep> frames;
    const int64_t epoch = 1700000000000000000LL;
    const double sector = 1e9 / 12.0 / 24.0;
    for (int turn = 0; turn < 7; ++turn) {
        for (int block = 0; block < 24; ++block) {
            const size_t samples = 67 + (block + turn) % 5;
            auto bytes = packet(block * 15, samples, 3);
            // A zero range is an actual slot, unlike FFFF padding.
            if (block == 6) {
                bytes[8] = bytes[9] = 0;
            }
            const double wire = bytes.size() * 10.0e9 / 500000;
            const int64_t received = epoch + std::llround((turn * 24 + block + 1.0 - 1.0 / samples) * sector + wire);
            const auto produced = assembler.consume(bytes.data(), bytes.size(), received, 500000, false);
            frames.insert(frames.end(), produced.begin(), produced.end());
        }
    }
    check(frames.size() == 5, "startup partial revolution or complete sweep count wrong");
    for (size_t index = 0; index < frames.size(); ++index) {
        const auto &scan = frames[index];
        check(scan.ranges.size() == 1800, "variable geometry leaked into output");
        check(scan.period > .0832 && scan.period < .0835, "wrong sweep period");
        check(scan.raw_samples >= 1600 && scan.raw_samples < 1800, "wrong physical return count");
        check(scan.first_ray_ns < epoch + static_cast<int64_t>((index + 2) * 1e9 / 12), "stamp is not first array ray");
        if (index) {
            check(scan.first_ray_ns > frames[index - 1].first_ray_ns, "nonmonotonic timestamp");
        }
        size_t finite = 0;
        for (float value : scan.ranges) {
            check(std::isinf(value) || value == 2.0f, "unobserved cells must be infinity, never zero");
            finite += std::isfinite(value);
        }
        check(finite > 1500 && finite < 1800, "no interpolation across empty angular cells");
        check(std::isinf(scan.ranges[1349]), "zero-distance return must not become an obstacle");
    }
    // A host clock regression and long receive gap must discard the partial sweep.
    const auto bytes = packet(0, 70);
    check(assembler.consume(bytes.data(), bytes.size(), epoch - 1, 500000, false).empty(), "clock jump published stale data");
    check(assembler.consume(bytes.data(), bytes.size(), epoch + 10000000000LL, 500000, false).empty(), "receive gap published stale data");
}

int main() {
    try {
        decode_tests();
        sweep_tests();
        std::cout << "PASS M10P padding, variable packets, fixed grid, startup discard, timing, empty cells and clock reset\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
