// SPDX-License-Identifier: Apache-2.0
#include "port_handler_mac.h"
#include <IOKit/serial/ioss.h>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

PortHandlerMac::PortHandlerMac(const char* name) {
  is_using_ = false;
  setPortName(name);
}

void PortHandlerMac::setPortName(const char* name) {
  if (!name || std::strlen(name) >= sizeof(name_))
    throw std::invalid_argument("シリアルポート名が不正です");
  std::strcpy(name_, name);
}

void PortHandlerMac::closePort() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

void PortHandlerMac::clearPort() {
  if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

bool PortHandlerMac::setBaudRate(int baudrate) {
  closePort();
  if (baudrate <= 0) return false;
  fd_ = ::open(name_, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) return false;
  termios settings{};
  bool ok = tcgetattr(fd_, &settings) == 0;
  if (ok) {
    cfmakeraw(&settings);
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = settings.c_cc[VTIME] = 0;
    ok = cfsetispeed(&settings, B38400) == 0 && cfsetospeed(&settings, B38400) == 0 &&
         tcsetattr(fd_, TCSANOW, &settings) == 0;
  }
  // termiosの標準定数にはない1 MbpsもIOSSIOSPEEDで設定する。
  speed_t speed = static_cast<speed_t>(baudrate);
  ok = ok && ioctl(fd_, IOSSIOSPEED, &speed) == 0;
  if (!ok) {
    closePort();
    return false;
  }
  baudrate_ = baudrate;
  clearPort();
  return true;
}

int PortHandlerMac::getBytesAvailable() {
  int count{};
  return ioctl(fd_, FIONREAD, &count) == 0 ? count : -1;
}

int PortHandlerMac::readPort(uint8_t* packet, int length) {
  return static_cast<int>(::read(fd_, packet, length));
}

int PortHandlerMac::writePort(uint8_t* packet, int length) {
  return static_cast<int>(::write(fd_, packet, length));
}

void PortHandlerMac::setPacketTimeout(uint16_t length) {
  setPacketTimeout(10000.0 * length / baudrate_ + 2.0 * latency_ms_ + 2.0);
}

void PortHandlerMac::setPacketTimeout(double milliseconds) {
  started_ = std::chrono::steady_clock::now();
  timeout_ms_ = milliseconds;
}

bool PortHandlerMac::isPacketTimeout() {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started_).count() > timeout_ms_;
}
