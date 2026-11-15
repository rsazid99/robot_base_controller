/*
 * @file            src/bno085.cpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 03:27:33
 * @lastModified    2026-08-08 03:27:33
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
#include "robot_base_controller/bno085.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace bno085
{
namespace
{

// SHTP channels
[[maybe_unused]] constexpr uint8_t CHANNEL_COMMAND = 0;   // advertisements, discarded
constexpr uint8_t CHANNEL_EXECUTABLE = 1;
constexpr uint8_t CHANNEL_CONTROL    = 2;
constexpr uint8_t CHANNEL_INPUT      = 3;
constexpr uint8_t CHANNEL_WAKE_INPUT = 4;
constexpr uint8_t CHANNEL_GYRO       = 5;

// SH-2 control reports. Note the product ID response is 0xF8, not 0xF1; 0xF1 is
// a command response, which the sensor also emits unsolicited after a reset.
[[maybe_unused]] constexpr uint8_t SHTP_REPORT_COMMAND_RESPONSE = 0xF1;
constexpr uint8_t SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8;
constexpr uint8_t SHTP_REPORT_PRODUCT_ID_REQUEST  = 0xF9;
constexpr uint8_t SHTP_REPORT_SET_FEATURE_COMMAND = 0xFD;

// Timestamp reports that prefix a batch of sensor reports
constexpr uint8_t REPORT_BASE_TIMESTAMP   = 0xFB;
constexpr uint8_t REPORT_TIMESTAMP_REBASE = 0xFA;

constexpr size_t HEADER_SIZE = 4;
constexpr size_t MAX_PACKET  = 512;

/// Fixed point conversion: raw * 2^-q
inline double qToDouble(int16_t raw, int q)
{
    return static_cast<double>(raw) * std::pow(2.0, -q);
}

inline int16_t read16(const uint8_t * p)
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}

}  // namespace

size_t reportLength(uint8_t report_id)
{
    switch (report_id) {
        case 0x01: return 10;  // accelerometer
        case 0x02: return 10;  // gyroscope calibrated
        case 0x03: return 10;  // magnetic field calibrated
        case 0x04: return 10;  // linear acceleration
        case 0x05: return 14;  // rotation vector
        case 0x06: return 10;  // gravity
        case 0x07: return 16;  // gyroscope uncalibrated
        case 0x08: return 12;  // game rotation vector
        case 0x09: return 14;  // geomagnetic rotation vector
        case 0x0A: return 8;   // pressure
        case 0x0B: return 8;   // ambient light
        case 0x0C: return 8;   // humidity
        case 0x0D: return 8;   // proximity
        case 0x0E: return 8;   // temperature
        case 0x0F: return 16;  // magnetic field uncalibrated
        case 0x10: return 5;   // tap detector
        case 0x11: return 12;  // step counter
        case 0x12: return 6;   // significant motion
        case 0x13: return 6;   // stability classifier
        case 0x14: return 16;  // raw accelerometer
        case 0x15: return 16;  // raw gyroscope
        case 0x16: return 16;  // raw magnetometer
        case 0x18: return 8;   // step detector
        case 0x19: return 6;   // shake detector
        case 0x28: return 14;  // ARVR stabilized rotation vector
        case 0x29: return 12;  // ARVR stabilized game rotation vector
        case REPORT_BASE_TIMESTAMP:   return 5;
        case REPORT_TIMESTAMP_REBASE: return 5;
        default: return 0;
    }
}

Bno085I2c::~Bno085I2c()
{
    close();
}

void Bno085I2c::open(int bus, uint8_t address, size_t max_transfer)
{
    close();

    max_transfer_ = std::clamp<size_t>(max_transfer, 16, 4096);
    rx_.assign(MAX_PACKET, 0);

    const std::string dev = "/dev/i2c-" + std::to_string(bus);

    fd_ = ::open(dev.c_str(), O_RDWR);
    if (fd_ < 0) {
        throw std::runtime_error("open(" + dev + ") failed: " + std::strerror(errno));
    }

    if (ioctl(fd_, I2C_SLAVE, static_cast<int>(address)) < 0) {
        const auto error = std::string("I2C_SLAVE 0x") + std::to_string(address) +
                           " failed: " + std::strerror(errno);
        close();
        throw std::runtime_error(error);
    }

    seq_.fill(0);
}

void Bno085I2c::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool Bno085I2c::sendPacket(uint8_t channel, const uint8_t * payload, size_t len)
{
    if (fd_ < 0 || channel >= seq_.size()) {
        last_error_ = "device not open";
        return false;
    }

    const size_t total = len + HEADER_SIZE;
    if (total > max_transfer_) {
        last_error_ = "packet larger than max_transfer";
        return false;
    }

    std::vector<uint8_t> buf(total);
    buf[0] = static_cast<uint8_t>(total & 0xFF);
    buf[1] = static_cast<uint8_t>((total >> 8) & 0xFF);
    buf[2] = channel;
    buf[3] = seq_[channel]++;
    std::memcpy(buf.data() + HEADER_SIZE, payload, len);

    const ssize_t n = ::write(fd_, buf.data(), buf.size());
    if (n != static_cast<ssize_t>(buf.size())) {
        last_error_ = std::string("i2c write failed: ") + std::strerror(errno);
        return false;
    }

    return true;
}

size_t Bno085I2c::readPacket()
{
    if (fd_ < 0) {
        return 0;
    }

    uint8_t header[HEADER_SIZE];
    if (::read(fd_, header, HEADER_SIZE) != static_cast<ssize_t>(HEADER_SIZE)) {
        last_error_ = std::string("i2c header read failed: ") + std::strerror(errno);
        return 0;
    }

    uint16_t packet_size = static_cast<uint16_t>(header[0]) |
                           (static_cast<uint16_t>(header[1]) << 8);

    // Bit 15 marks a continuation of a previous transfer, not part of the size.
    packet_size &= 0x7FFF;

    if (packet_size <= HEADER_SIZE) {
        return 0;  // nothing pending
    }

    if (packet_size > MAX_PACKET) {
        // Oversized cargo. Drain it in chunks so the sensor is not left mid-packet.
        last_error_ = "oversized SHTP packet (" + std::to_string(packet_size) + " bytes)";

        std::vector<uint8_t> sink(max_transfer_);
        size_t remaining = packet_size;
        bool first = true;

        while (remaining > 0) {
            const size_t want = first
                ? std::min<size_t>(max_transfer_, remaining)
                : std::min<size_t>(max_transfer_, remaining + HEADER_SIZE);

            if (::read(fd_, sink.data(), want) != static_cast<ssize_t>(want)) {
                return 0;
            }

            const size_t got = first ? want : want - HEADER_SIZE;
            remaining -= std::min(remaining, got);
            first = false;
        }

        return 0;
    }

    rx_.assign(packet_size, 0);

    // Re-read the packet from the start; the sensor repeats the 4 byte header at
    // the beginning of every read transaction, so continued reads skip it.
    size_t remaining = packet_size;
    size_t offset = 0;
    bool first = true;
    std::vector<uint8_t> chunk(max_transfer_);

    while (remaining > 0) {
        const size_t want = first
            ? std::min<size_t>(max_transfer_, remaining)
            : std::min<size_t>(max_transfer_, remaining + HEADER_SIZE);

        if (::read(fd_, chunk.data(), want) != static_cast<ssize_t>(want)) {
            last_error_ = std::string("i2c cargo read failed: ") + std::strerror(errno);
            return 0;
        }

        const size_t got = first ? want : want - HEADER_SIZE;
        const uint8_t * src = first ? chunk.data() : chunk.data() + HEADER_SIZE;

        if (got == 0 || got > remaining) {
            last_error_ = "malformed cargo continuation";
            return 0;
        }

        std::memcpy(rx_.data() + offset, src, got);
        offset += got;
        remaining -= got;
        first = false;
    }

    rx_channel_ = rx_[2];

    const size_t payload_size = packet_size - HEADER_SIZE;

    if (rx_channel_ == CHANNEL_CONTROL) {
        captureProductId(rx_.data() + HEADER_SIZE, payload_size);
    }

    return payload_size;
}

bool Bno085I2c::softReset()
{
    const uint8_t reset_cmd = 0x01;

    if (!sendPacket(CHANNEL_EXECUTABLE, &reset_cmd, 1)) {
        return false;
    }

    // The sensor reboots and then emits an advertisement plus a few
    // unsolicited responses. Give it time, then drain whatever is queued.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    seq_.fill(0);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
    bool saw_traffic = false;

    while (std::chrono::steady_clock::now() < deadline) {
        if (readPacket() > 0) {
            saw_traffic = true;
            continue;
        }

        if (saw_traffic) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!saw_traffic) {
        last_error_ = "no SHTP traffic after reset";
    }

    return saw_traffic;
}

void Bno085I2c::captureProductId(const uint8_t * payload, size_t len)
{
    if (len < 16 || payload[0] != SHTP_REPORT_PRODUCT_ID_RESPONSE) {
        return;
    }

    // The sensor answers with one response per subsystem (SH-2 firmware first,
    // then the bootloader). Keep the first, which is the interesting one.
    if (!product_info_.empty()) {
        return;
    }

    const uint32_t part_number =
        static_cast<uint32_t>(payload[4]) |
        (static_cast<uint32_t>(payload[5]) << 8) |
        (static_cast<uint32_t>(payload[6]) << 16) |
        (static_cast<uint32_t>(payload[7]) << 24);

    const uint32_t build =
        static_cast<uint32_t>(payload[8]) |
        (static_cast<uint32_t>(payload[9]) << 8) |
        (static_cast<uint32_t>(payload[10]) << 16) |
        (static_cast<uint32_t>(payload[11]) << 24);

    std::ostringstream oss;
    oss << "part " << part_number
        << ", firmware " << static_cast<int>(payload[2])
        << "." << static_cast<int>(payload[3])
        << "." << (static_cast<uint16_t>(payload[12]) |
                   (static_cast<uint16_t>(payload[13]) << 8))
        << ", build " << build
        << ", reset cause " << static_cast<int>(payload[1]);

    product_info_ = oss.str();
}

bool Bno085I2c::readProductId(std::string & info)
{
    if (!product_info_.empty()) {
        info = product_info_;
        return true;
    }

    const uint8_t req[2] = {SHTP_REPORT_PRODUCT_ID_REQUEST, 0x00};

    for (int attempt = 0; attempt < 3; attempt++) {
        if (!sendPacket(CHANNEL_CONTROL, req, sizeof(req))) {
            return false;
        }

        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(250);

        while (std::chrono::steady_clock::now() < deadline) {
            // readPacket() caches any product ID response it passes over.
            if (readPacket() == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            if (!product_info_.empty()) {
                info = product_info_;
                return true;
            }
        }
    }

    last_error_ = "timed out waiting for product ID response";
    return false;
}

bool Bno085I2c::enableReport(uint8_t report_id, uint32_t interval_us)
{
    uint8_t cmd[17] = {};
    cmd[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    cmd[1] = report_id;
    // cmd[2]   feature flags
    // cmd[3-4] change sensitivity
    cmd[5] = static_cast<uint8_t>(interval_us & 0xFF);
    cmd[6] = static_cast<uint8_t>((interval_us >> 8) & 0xFF);
    cmd[7] = static_cast<uint8_t>((interval_us >> 16) & 0xFF);
    cmd[8] = static_cast<uint8_t>((interval_us >> 24) & 0xFF);
    // cmd[9-12]  batch interval
    // cmd[13-16] sensor specific config

    if (!sendPacket(CHANNEL_CONTROL, cmd, sizeof(cmd))) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return true;
}

unsigned Bno085I2c::parseInputReport(const uint8_t * payload, size_t len, ImuSample & sample)
{
    unsigned updated = UPDATED_NONE;
    size_t idx = 0;

    // A batch of sensor reports is prefixed with a base timestamp report.
    if (len >= 5 && (payload[0] == REPORT_BASE_TIMESTAMP ||
                     payload[0] == REPORT_TIMESTAMP_REBASE))
    {
        idx = 5;
    }

    while (idx < len) {
        const uint8_t id = payload[idx];
        const size_t report_len = reportLength(id);

        if (report_len == 0 || idx + report_len > len) {
            // Unknown report: the rest of the batch cannot be located safely.
            break;
        }

        const uint8_t * r = payload + idx;
        const uint8_t status = r[2] & 0x03;

        switch (id) {
            case REPORT_ACCELEROMETER:
                sample.ax = qToDouble(read16(r + 4), 8);
                sample.ay = qToDouble(read16(r + 6), 8);
                sample.az = qToDouble(read16(r + 8), 8);
                sample.accel_status = status;
                updated |= UPDATED_ACCEL;
                break;

            case REPORT_GYROSCOPE:
                sample.gx = qToDouble(read16(r + 4), 9);
                sample.gy = qToDouble(read16(r + 6), 9);
                sample.gz = qToDouble(read16(r + 8), 9);
                sample.gyro_status = status;
                updated |= UPDATED_GYRO;
                break;

            case REPORT_MAGNETOMETER:
                sample.mx = qToDouble(read16(r + 4), 4);
                sample.my = qToDouble(read16(r + 6), 4);
                sample.mz = qToDouble(read16(r + 8), 4);
                sample.mag_status = status;
                updated |= UPDATED_MAG;
                break;

            case REPORT_ROTATION_VECTOR:
            case REPORT_GEOMAG_ROTATION_VEC:
                sample.qi = qToDouble(read16(r + 4), 14);
                sample.qj = qToDouble(read16(r + 6), 14);
                sample.qk = qToDouble(read16(r + 8), 14);
                sample.qr = qToDouble(read16(r + 10), 14);
                sample.quat_accuracy_rad = qToDouble(read16(r + 12), 12);
                sample.quat_status = status;
                updated |= UPDATED_QUAT;
                break;

            case REPORT_GAME_ROTATION_VECTOR:
                sample.qi = qToDouble(read16(r + 4), 14);
                sample.qj = qToDouble(read16(r + 6), 14);
                sample.qk = qToDouble(read16(r + 8), 14);
                sample.qr = qToDouble(read16(r + 10), 14);
                sample.quat_accuracy_rad = 0.0;
                sample.quat_status = status;
                updated |= UPDATED_QUAT;
                break;

            default:
                break;  // known length, not a report we use
        }

        idx += report_len;
    }

    return updated;
}

unsigned Bno085I2c::service(ImuSample & sample)
{
    unsigned updated = UPDATED_NONE;

    // Bound the work per call so a backlog cannot stall the caller forever.
    for (int i = 0; i < 32; i++) {
        const size_t len = readPacket();

        if (len == 0) {
            break;
        }

        if (rx_channel_ == CHANNEL_INPUT ||
            rx_channel_ == CHANNEL_WAKE_INPUT ||
            rx_channel_ == CHANNEL_GYRO)
        {
            updated |= parseInputReport(rx_.data() + HEADER_SIZE, len, sample);
        }

        // Control and command channel traffic is not needed once streaming;
        // readPacket() already picks the product ID out of it.
    }

    return updated;
}

}  // namespace bno085
