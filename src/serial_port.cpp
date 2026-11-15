/*
 * @file            src/serial_port.cpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 03:27:33
 * @lastModified    2026-08-08 03:27:33
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
#include "robot_base_controller/serial_port.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace robot_base
{

SerialPort::~SerialPort()
{
    close();
}

void SerialPort::open(const std::string & device, speed_t baud)
{
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        throw std::runtime_error(
            "open(" + device + ") failed: " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        const auto error = std::string("tcgetattr failed: ") + std::strerror(errno);
        close();
        throw std::runtime_error(error);
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        const auto error = std::string("tcsetattr failed: ") + std::strerror(errno);
        close();
        throw std::runtime_error(error);
    }

    tcflush(fd_, TCIOFLUSH);
}

void SerialPort::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void SerialPort::flushInput()
{
    if (fd_ >= 0) {
        tcflush(fd_, TCIFLUSH);
    }
}

void SerialPort::write(const uint8_t * data, size_t size)
{
    size_t written = 0;

    while (written < size) {
        const ssize_t n = ::write(fd_, data + written, size - written);

        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        throw std::runtime_error(std::string("serial write failed: ") + std::strerror(errno));
    }

    tcdrain(fd_);
}

size_t SerialPort::read(uint8_t * data, size_t size)
{
    const ssize_t n = ::read(fd_, data, size);

    if (n > 0) {
        return static_cast<size_t>(n);
    }

    if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return 0;
    }

    throw std::runtime_error(std::string("serial read failed: ") + std::strerror(errno));
}

size_t SerialPort::available() const
{
    if (fd_ < 0) {
        return 0;
    }

    int bytes = 0;
    if (ioctl(fd_, FIONREAD, &bytes) < 0 || bytes < 0) {
        return 0;
    }

    return static_cast<size_t>(bytes);
}

speed_t baudToSpeed(int baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            throw std::runtime_error("unsupported baud rate: " + std::to_string(baud));
    }
}

bool enableRs485Mode(int fd)
{
    if (fd < 0) {
        return false;
    }

    serial_rs485 rs485{};
    rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
    rs485.delay_rts_before_send = 0;
    rs485.delay_rts_after_send = 0;

    return ioctl(fd, TIOCSRS485, &rs485) >= 0;
}

}  // namespace robot_base
