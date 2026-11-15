/*
 * @file            include/robot_base_controller/serial_port.hpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 03:27:33
 * @lastModified    2026-08-08 03:27:33
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
#pragma once

#include <termios.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace robot_base
{

/// Thin RAII wrapper over a non-blocking raw serial port.
class SerialPort
{
public:
    SerialPort() = default;
    SerialPort(const SerialPort &) = delete;
    SerialPort & operator=(const SerialPort &) = delete;
    ~SerialPort();

    /// Opens the device in raw, non-blocking mode. Throws std::runtime_error on failure.
    void open(const std::string & device, speed_t baud = B115200);

    bool isOpen() const { return fd_ >= 0; }
    int fd() const { return fd_; }

    void close();
    void flushInput();

    /// Blocking write of the whole buffer. Throws std::runtime_error on failure.
    void write(const uint8_t * data, size_t size);

    /// Non-blocking read. Returns the number of bytes read, 0 if none are pending.
    size_t read(uint8_t * data, size_t size);

    /// Number of bytes waiting in the kernel input buffer (0 on error).
    size_t available() const;

private:
    int fd_ = -1;
};

/// Maps a numeric baud rate to its termios constant. Throws on unsupported values.
speed_t baudToSpeed(int baud);

/// Requests kernel RS485 half-duplex mode. Returns false if the driver does not support it.
bool enableRs485Mode(int fd);

}  // namespace robot_base
