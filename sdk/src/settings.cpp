// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/settings.hpp"
#include "dynamixel_communicator.h"
#include <set>
#include <stdexcept>
namespace fuzzrobo::topoarm_hardware {
void validate_settings(const std::vector<Joint> &joints, const SessionOptions &options) {
    if (options.read_profile != "teleop" && options.read_profile != "bam")
        throw std::invalid_argument("read_profileにはteleopまたはbamを指定してください");
    if (!std::isfinite(options.floor_z_m) || !std::isfinite(options.floor_margin_m) ||
        options.floor_margin_m < .001 || (options.floor_enabled && options.robot_model.empty()))
        throw std::invalid_argument("床設定には有限の高さ・1mm以上の余裕・有効時のURDFが必要です");
    if (options.return_delay_time < -1 || options.return_delay_time > 254)
        throw std::invalid_argument("Return Delay Timeが不正です");
    for (int value : {options.xm540_p, options.xm540_d, options.xm430_p, options.xm430_d})
        if (value < -1 || value > 16383)
            throw std::invalid_argument("モデル別P/Dは0〜16383です");
    for (int value :
         {options.xm540_profile_ms, options.xm540_accel_ms, options.xm430_profile_ms, options.xm430_accel_ms})
        if (value < -1 || value > 32737)
            throw std::invalid_argument("モデル別時間は0〜32737msです");
    if ((options.xm540_profile_ms >= 0 && options.xm540_accel_ms > options.xm540_profile_ms / 2) ||
        (options.xm430_profile_ms >= 0 && options.xm430_accel_ms > options.xm430_profile_ms / 2))
        throw std::invalid_argument("加速時間は全体時間の半分以下にしてください");
    if (joints.empty())
        throw std::invalid_argument("対象関節が空です");
    if (options.neck_envelope)
        NeckGuard::limits(joints, options.neck_limits);
    std::set<int> ids;
    std::set<std::string> names;
    for (const auto &d : joints) {
        const bool timed = d.config.profile_duration_ms != -1 || d.config.profile_acceleration_ms != -1;
        if (timed && (d.config.profile_duration_ms < 0 || d.config.profile_duration_ms > 32737 ||
                      d.config.profile_acceleration_ms < 0 ||
                      d.config.profile_acceleration_ms > d.config.profile_duration_ms / 2 ||
                      d.config.profile_vel != 0 || d.config.profile_acc != 0))
            throw std::invalid_argument(
                "時間プロファイルは両値をmsで指定し、加速時間を全体の半分以下にしてください: " +
                d.config.joint_name);
        if (d.config.id == 0 || d.config.id > 253 || !ids.insert(d.config.id).second ||
            d.config.joint_name.empty() || !names.insert(d.config.joint_name).second ||
            (d.config.operating_mode != 3 && d.config.operating_mode != 5) ||
            !std::isfinite(d.config.multiplier) || d.config.multiplier == 0 ||
            !std::isfinite(d.config.offset) || !std::isfinite(d.config.limit_pos_min) ||
            !std::isfinite(d.config.limit_pos_max) || d.config.limit_pos_min >= d.config.limit_pos_max ||
            !std::isfinite(d.command.current_limit) || !std::isfinite(d.config.current_cap) ||
            d.command.current_limit < 0 || d.config.current_cap <= 0 ||
            d.command.current_limit > d.config.current_cap || !std::isfinite(d.config.profile_vel) ||
            !std::isfinite(d.config.profile_acc) || d.config.profile_vel < 0 || d.config.profile_acc < 0 ||
            d.config.position_kp < 0 || d.config.position_kp > 16383 || d.config.position_ki < 0 ||
            d.config.position_ki > 16383 || d.config.position_kd < 0 || d.config.position_kd > 16383)
            throw std::invalid_argument("関節設定が不正です: " + d.config.joint_name);
    }
}
void apply_model_overrides(std::vector<Joint> &joints, const SessionOptions &options) {
    // 実機型番で決める。指・首も含み、関節名や配列の位置では分類しない。
    for (auto &d : joints) {
        const bool xm540 =
            d.state.model_number == MODEL_XM540_W270 || d.state.model_number == MODEL_XM540_W150;
        const bool xm430 =
            d.state.model_number == MODEL_XM430_W350 || d.state.model_number == MODEL_XM430_W210;
        if (!xm540 && !xm430)
            continue;
        const int p = xm540 ? options.xm540_p : options.xm430_p;
        const int kd = xm540 ? options.xm540_d : options.xm430_d;
        const int duration = xm540 ? options.xm540_profile_ms : options.xm430_profile_ms;
        const int accel = xm540 ? options.xm540_accel_ms : options.xm430_accel_ms;
        if (p >= 0)
            d.config.position_kp = p;
        if (kd >= 0)
            d.config.position_kd = kd;
        if (duration >= 0)
            d.config.profile_duration_ms = duration;
        if (accel >= 0)
            d.config.profile_acceleration_ms = accel;
        if (duration >= 0 || accel >= 0) {
            d.config.profile_vel = 0;
            d.config.profile_acc = 0;
        }
    }
}
} // namespace fuzzrobo::topoarm_hardware
