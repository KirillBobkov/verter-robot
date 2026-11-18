// SPDX-License-Identifier: GPL-3.0-or-later
// M10P packet decoding and fixed angular grid. No ROS, threads or device commands.
#ifndef LSLIDAR_M10P_SCAN_H
#define LSLIDAR_M10P_SCAN_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace lslidar_driver {

struct M10pSample {
    double degree;
    float range;
    float intensity;
};

inline std::vector<M10pSample> decode_m10p(const uint8_t *bytes, size_t size,
                                        bool high_reflection, double angle_offset = 0.0) {
    if (size < 22 || size > 500 || (size - 20) % 2 != 0 ||
        bytes[0] != 0xa5 || bytes[1] != 0x5a ||
        (static_cast<size_t>(bytes[2]) * 256 + bytes[3]) != size ||
        bytes[size - 2] != 0xfa || bytes[size - 1] != 0xfb || !std::isfinite(angle_offset)) {
        throw std::invalid_argument("invalid M10P packet framing");
    }
    const unsigned start = bytes[4] * 256u + bytes[5];
    if (start >= 36000) {
        throw std::invalid_argument("invalid M10P encoder angle");
    }
    std::vector<uint16_t> values;
    for (size_t index = 8; index < size - 12; index += 2) {
        const uint16_t raw = bytes[index] * 256u + bytes[index + 1];
        // Vendor protocol: FFFF words are padding, not angular sample slots.
        if (raw != 0xffff) {
            values.push_back(raw);
        }
    }
    std::vector<M10pSample> samples;
    samples.reserve(values.size());
    for (size_t index = 0; index < values.size(); ++index) {
        double degree = std::fmod(start / 100.0 + angle_offset + 15.0 * index / values.size(), 360.0);
        if (degree < 0) {
            degree += 360.0;
        }
        const auto raw = values[index];
        samples.push_back({degree, static_cast<float>((high_reflection ? raw & 0x7fff : raw) / 1000.0),
                           high_reflection && (raw & 0x8000) ? 255.0f : 0.0f});
    }
    return samples;
}

struct M10pSweep {
    int64_t first_ray_ns;
    double period;
    size_t raw_samples;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

class M10pScanAssembler {
public:
    M10pScanAssembler(size_t bins, float range_min, float range_max)
        : bins_(bins), range_min_(range_min), range_max_(range_max) {
        if (bins < 360 || bins > 10000 || !std::isfinite(range_min) || !std::isfinite(range_max) ||
            range_min < 0 || range_min >= range_max) {
            throw std::invalid_argument("invalid M10P grid or range bounds");
        }
        clear_grid();
    }

    std::vector<M10pSweep> consume(const uint8_t *bytes, size_t size, int64_t receipt_ns,
                                  unsigned serial_baud, bool high_reflection, double angle_offset = 0.0) {
        const auto samples = decode_m10p(bytes, size, high_reflection, angle_offset);
        std::vector<M10pSweep> completed;
        if (receipt_ns <= 0 || (last_receipt_ &&
            (receipt_ns <= last_receipt_ || receipt_ns - last_receipt_ > std::max(50000000.0, period_ns_ * 0.75)))) {
            reset();
        }
        if (receipt_ns <= 0) {
            return completed;
        }
        last_receipt_ = receipt_ns;
        if (samples.empty()) {
            reset();
            return completed;
        }
        // Host receive time is the end of the packet. Estimate first sample time,
        // removing the 8N1 wire duration and the remaining sector acquisition time.
        // USB buffering and device processing latency still require calibration.
        const double wire_ns = serial_baud ? size * 10.0e9 / serial_baud : 0.0;
        const double sector_ns = period_ns_ / 24.0;
        const double first_ns = receipt_ns - wire_ns - sector_ns * (samples.size() - 1) / samples.size();
        for (size_t index = 0; index < samples.size(); ++index) {
            const auto &sample = samples[index];
            const double sample_ns = first_ns + sector_ns * index / samples.size();
            if (have_phase_) {
                const double change = sample.degree - last_phase_;
                if (last_phase_ > 300 && sample.degree < 60 && change < -180) {
                    const int64_t boundary = std::llround(sample_ns - sample.degree * period_ns_ / 360.0);
                    const int64_t duration = boundary - sweep_start_;
                    if (synchronized_ && duration >= 40000000 && duration <= 250000000 && raw_samples_ >= 100) {
                        // Native encoder angles increase clockwise. ROS angles increase
                        // counterclockwise: array order runs backwards through acquisition.
                        // Cell centers exclude a duplicated 0/360 endpoint.
                        completed.push_back({boundary - duration / static_cast<int64_t>(2 * bins_),
                                             duration / 1.0e9, raw_samples_, ranges_, intensities_});
                        period_ns_ = duration;
                    }
                    clear_grid();
                    sweep_start_ = boundary;
                    synchronized_ = true;
                } else if (change < -0.01 || change > 90) {
                    // Out-of-order or very incomplete data must not join two revolutions.
                    synchronized_ = false;
                    clear_grid();
                }
            }
            have_phase_ = true;
            last_phase_ = sample.degree;
            if (!synchronized_) {
                continue;  // Discard the partial revolution present at startup.
            }
            ++raw_samples_;
            const size_t native = std::min(bins_ - 1, static_cast<size_t>(sample.degree * bins_ / 360.0));
            const size_t output = bins_ - 1 - native;
            if (std::isfinite(sample.range) && sample.range >= range_min_ && sample.range <= range_max_ &&
                sample.range < ranges_[output]) {
                // Keep the nearest return when multiple measurements hit the same cell.
                ranges_[output] = sample.range;
                intensities_[output] = sample.intensity;
            }
        }
        return completed;
    }

private:
    void clear_grid() {
        ranges_.assign(bins_, std::numeric_limits<float>::infinity());
        intensities_.assign(bins_, 0.0f);
        raw_samples_ = 0;
    }

    void reset() {
        have_phase_ = synchronized_ = false;
        period_ns_ = 1.0e9 / 12.0;
        last_receipt_ = 0;
        clear_grid();
    }

    size_t bins_;
    float range_min_, range_max_;
    std::vector<float> ranges_, intensities_;
    size_t raw_samples_ = 0;
    bool have_phase_ = false;
    bool synchronized_ = false;
    double last_phase_ = 0.0;
    double period_ns_ = 1.0e9 / 12.0;
    int64_t sweep_start_ = 0;
    int64_t last_receipt_ = 0;
};

}  // namespace lslidar_driver
#endif
