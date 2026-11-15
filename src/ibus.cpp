/*
 * @file            src/ibus.cpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 03:27:33
 * @lastModified    2026-08-08 03:27:33
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
#include "robot_base_controller/ibus.hpp"

#include <algorithm>
#include <cmath>

namespace ibus
{

bool checksumOk(const uint8_t * frame)
{
    uint16_t checksum = 0xFFFF;

    for (size_t i = 0; i < FRAME_LEN - 2; i++) {
        checksum -= frame[i];
    }

    const uint16_t rx = static_cast<uint16_t>(frame[FRAME_LEN - 2]) |
                        (static_cast<uint16_t>(frame[FRAME_LEN - 1]) << 8);

    return checksum == rx;
}

std::optional<Channels> parseFrame(const uint8_t * frame)
{
    if (frame[0] != 0x20 || frame[1] != 0x40) {
        return std::nullopt;
    }

    if (!checksumOk(frame)) {
        return std::nullopt;
    }

    Channels channels{};

    for (size_t i = 0; i < NUM_CHANNELS; i++) {
        channels[i] = static_cast<uint16_t>(frame[2 + i * 2]) |
                      (static_cast<uint16_t>(frame[3 + i * 2]) << 8);
    }

    return channels;
}

std::optional<Channels> extractLatest(std::vector<uint8_t> & buffer)
{
    std::optional<Channels> latest;
    size_t pos = 0;

    while (buffer.size() - pos >= FRAME_LEN) {
        if (buffer[pos] != 0x20 || buffer[pos + 1] != 0x40) {
            pos++;  // resync one byte at a time
            continue;
        }

        auto channels = parseFrame(buffer.data() + pos);

        if (channels) {
            latest = channels;
            pos += FRAME_LEN;
        } else {
            // Header matched but the checksum did not: corrupt bytes, resync.
            pos++;
        }
    }

    buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(pos));

    // Never let partial junk grow without bound.
    if (buffer.size() > 4 * FRAME_LEN) {
        buffer.clear();
    }

    return latest;
}

double mapChannel(uint16_t value, double max_out, int center, int deadband)
{
    if (value < 900 || value > 2100) {
        return 0.0;
    }

    const double diff = static_cast<double>(value) - static_cast<double>(center);

    if (std::abs(diff) < static_cast<double>(deadband)) {
        return 0.0;
    }

    const double usable_range = 500.0 - static_cast<double>(deadband);

    if (usable_range <= 0.0) {
        return 0.0;
    }

    const double magnitude = (diff > 0.0)
        ? (diff - deadband) / usable_range
        : (diff + deadband) / usable_range;

    return std::clamp(magnitude, -1.0, 1.0) * max_out;
}

double mapKnob(uint16_t value, double min_out, double max_out)
{
    const double clamped = std::clamp(static_cast<double>(value), 1000.0, 2000.0);
    const double ratio = (clamped - 1000.0) / 1000.0;
    return min_out + ratio * (max_out - min_out);
}

}  // namespace ibus
