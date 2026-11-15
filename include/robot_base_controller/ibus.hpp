/*
 * @file            include/robot_base_controller/ibus.hpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 02:40:11
 * @lastModified    2026-08-08 02:40:11
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace ibus
{

static constexpr size_t FRAME_LEN     = 32;
static constexpr size_t NUM_CHANNELS  = 14;

using Channels = std::array<uint16_t, NUM_CHANNELS>;

/// Verifies the trailing 16 bit i-BUS checksum of a full 32 byte frame.
bool checksumOk(const uint8_t * frame);

/// Parses a full 32 byte i-BUS servo frame (0x20 0x40 header).
std::optional<Channels> parseFrame(const uint8_t * frame);

/// Consumes complete frames from `buffer` and returns the newest valid one,
/// so a backlog of buffered frames never adds control latency.
/// Bytes belonging to an incomplete trailing frame are kept in the buffer.
std::optional<Channels> extractLatest(std::vector<uint8_t> & buffer);

/// Maps a channel value centred on 1500 to [-max_out, +max_out].
double mapChannel(uint16_t value, double max_out, int center = 1500, int deadband = 40);

/// Maps a 1000..2000 knob channel to [min_out, max_out].
double mapKnob(uint16_t value, double min_out, double max_out);

}  // namespace ibus
