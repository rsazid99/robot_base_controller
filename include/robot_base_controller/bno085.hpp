/*
 * @file            include/robot_base_controller/bno085.hpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 02:51:46
 * @lastModified    2026-08-08 02:51:46
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace bno085
{

// SH-2 sensor report IDs (the ones this driver understands).
static constexpr uint8_t REPORT_ACCELEROMETER        = 0x01;
static constexpr uint8_t REPORT_GYROSCOPE            = 0x02;
static constexpr uint8_t REPORT_MAGNETOMETER         = 0x03;
static constexpr uint8_t REPORT_LINEAR_ACCELERATION  = 0x04;
static constexpr uint8_t REPORT_ROTATION_VECTOR      = 0x05;
static constexpr uint8_t REPORT_GRAVITY              = 0x06;
static constexpr uint8_t REPORT_GAME_ROTATION_VECTOR = 0x08;
static constexpr uint8_t REPORT_GEOMAG_ROTATION_VEC  = 0x09;

/// Bit flags returned by service() describing which fields were refreshed.
enum Updated : unsigned
{
    UPDATED_NONE  = 0u,
    UPDATED_ACCEL = 1u << 0,
    UPDATED_GYRO  = 1u << 1,
    UPDATED_MAG   = 1u << 2,
    UPDATED_QUAT  = 1u << 3
};

/// Latest values from the sensor. Fields keep their previous value until refreshed.
struct ImuSample
{
    double ax = 0.0, ay = 0.0, az = 0.0;   // m/s^2
    double gx = 0.0, gy = 0.0, gz = 0.0;   // rad/s
    double mx = 0.0, my = 0.0, mz = 0.0;   // microtesla
    double qi = 0.0, qj = 0.0, qk = 0.0, qr = 1.0;

    double  quat_accuracy_rad = 0.0;   // only reported by the full rotation vector
    uint8_t accel_status = 0;          // 0 unreliable .. 3 high accuracy
    uint8_t gyro_status  = 0;
    uint8_t mag_status   = 0;
    uint8_t quat_status  = 0;
};

/// Minimal BNO08x / BNO085 driver speaking SHTP over Linux i2c-dev.
///
/// Only the pieces needed for an IMU stream are implemented: soft reset,
/// product ID query, "set feature" to enable periodic reports, and parsing of
/// the input reports on the normal sensor channel.
class Bno085I2c
{
public:
    Bno085I2c() = default;
    Bno085I2c(const Bno085I2c &) = delete;
    Bno085I2c & operator=(const Bno085I2c &) = delete;
    ~Bno085I2c();

    /// Opens /dev/i2c-<bus> and selects the device address.
    /// `max_transfer` bounds a single I2C transaction; larger SHTP packets are
    /// read as continued cargo. Throws std::runtime_error on failure.
    void open(int bus, uint8_t address, size_t max_transfer = 64);

    void close();
    bool isOpen() const { return fd_ >= 0; }

    /// Issues a soft reset and drains the advertisement traffic that follows.
    bool softReset();

    /// Queries the product ID. Populates `info` with a human readable summary.
    bool readProductId(std::string & info);

    /// Enables a periodic sensor report at the requested interval.
    bool enableReport(uint8_t report_id, uint32_t interval_us);

    /// Drains every packet the sensor has pending and merges the sensor reports
    /// into `sample`. Returns a bitwise OR of Updated flags.
    unsigned service(ImuSample & sample);

    /// Human readable description of the last failure.
    const std::string & lastError() const { return last_error_; }

private:
    bool sendPacket(uint8_t channel, const uint8_t * payload, size_t len);

    /// Caches a product ID response. The sensor sends one unsolicited after a
    /// reset, so every control packet is inspected rather than only replies.
    void captureProductId(const uint8_t * payload, size_t len);

    /// Reads one SHTP packet into rx_. Returns the payload size, or 0 when the
    /// sensor has nothing pending (or on a recoverable error).
    size_t readPacket();

    unsigned parseInputReport(const uint8_t * payload, size_t len, ImuSample & sample);

    int fd_ = -1;
    size_t max_transfer_ = 64;
    std::array<uint8_t, 6> seq_{};
    std::vector<uint8_t> rx_;
    uint8_t rx_channel_ = 0;
    std::string product_info_;
    std::string last_error_;
};

/// Length in bytes of an SH-2 input report, including its report ID.
/// Returns 0 for reports this driver does not know how to skip over.
size_t reportLength(uint8_t report_id);

}  // namespace bno085
