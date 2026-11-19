// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/neck_guard.hpp"
#include <string>
namespace fuzzrobo::topoarm_hardware {
struct SessionOptions {
    bool verified{false}, torque{false}, retention{true}, configure_devices{false};
    // 実測値は範囲外でも受け取り、起動時を含む目標角度だけを制限する。
    bool clip_to_limits{false};
    // Torque ON接続時、RAMの位置PIDを毎回書き込み読戻す。
    bool write_pid_on_connect{false};
    // -1は維持。0〜254は起動時に異なる値だけ変更する（2µs単位）。
    int return_delay_time{-1};
    // -1はYAMLを維持。実機のModel Numberで全関節へ適用する。
    int xm540_p{-1};
    int xm540_d{-1};
    int xm540_profile_ms{-1};
    int xm540_accel_ms{-1};
    int xm430_p{-1};
    int xm430_d{-1};
    int xm430_profile_ms{-1};
    int xm430_accel_ms{-1};
    bool neck_envelope{false};
    NeckEnvelopeConfig neck_limits;
    std::string robot_model;
    bool floor_enabled{false};
    double floor_z_m{0}, floor_margin_m{.02};
    // 運転中の電圧・過負荷エラーだけを、IDごとに接続中1回復旧する。
    bool auto_reset{false};
};
} // namespace fuzzrobo::topoarm_hardware
