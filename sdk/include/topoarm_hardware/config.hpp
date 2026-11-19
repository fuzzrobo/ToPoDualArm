// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/driver_session.hpp"
namespace fuzzrobo::topoarm_hardware {
// ファイルの読み込みと検証のみ。ポートを開かず、実機には書き込まない。
struct HardwareConfig {
    std::string port{"/dev/ttyDXL_u2d2"};
    int baud_rate{1000000}, latency_timer{1};
    bool hardware_verified{false};
    std::vector<JointConfig> joints;
    SessionOptions options;
    double neck_tilt_negative_scale{1.0};
};
HardwareConfig load_config(const std::string &path, const std::vector<std::string> &selected = {});
} // namespace fuzzrobo::topoarm_hardware
