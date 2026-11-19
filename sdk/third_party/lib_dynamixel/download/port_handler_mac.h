// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "port_handler_fix.h"
#include <chrono>

// 製品SDK共通のProtocol 2.0実装に接続するmacOSシリアルポート。
class PortHandlerMac final : public PortHandler {
 public:
  explicit PortHandlerMac(const char* name);
  ~PortHandlerMac() override { closePort(); }
  bool openPort() override { return setBaudRate(baudrate_); }
  void closePort() override;
  void clearPort() override;
  void setPortName(const char* name) override;
  char* getPortName() override { return name_; }
  // FTDIドライバ設定ではなく、応答待ち時間の計算値を設定する。
  void setLatencyTimer(int value) override { latency_ms_ = value; }
  int getLatencyTimer() override { return latency_ms_; }
  bool setBaudRate(int baudrate) override;
  int getBaudRate() override { return baudrate_; }
  int getBytesAvailable() override;
  int readPort(uint8_t* packet, int length) override;
  int writePort(uint8_t* packet, int length) override;
  void setPacketTimeout(uint16_t length) override;
  void setPacketTimeout(double milliseconds) override;
  bool isPacketTimeout() override;

 private:
  int fd_{-1}, baudrate_{DEFAULT_BAUDRATE_}, latency_ms_{16};
  char name_[100]{};
  std::chrono::steady_clock::time_point started_{};
  double timeout_ms_{};
};
