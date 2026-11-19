// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace fuzzrobo::topoarm_hardware {
struct JointConfig {
    std::string joint_name;
    uint8_t id{0};
    double multiplier{1.0}, offset{0.0};
    double profile_acc{7.5}, profile_vel{0.5};
    int profile_duration_ms{-1}, profile_acceleration_ms{-1};
    int position_kp{800}, position_ki{0}, position_kd{0};
    double limit_pos_min{-3.141592653589793}, limit_pos_max{3.141592653589793};
    double current_cap{0.1}, initial_current_limit{0.0};
    uint8_t operating_mode{5};
};
struct JointState {
    // スナップショットを関節名で対応付ける。
    std::string name;
    uint16_t model_number{0};
    int64_t position_raw{0};
    double position{std::numeric_limits<double>::quiet_NaN()};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double current{std::numeric_limits<double>::quiet_NaN()};
    double pwm{0.0}, voltage{0.0}, temperature{0.0};
    double valid{0.0}, age{0.0};
    double hardware_error{0.0}, torque_enabled{0.0}, realtime_tick{0.0};
    double goal_position_readback{0.0};
    double position_trajectory{std::numeric_limits<double>::quiet_NaN()};
    double velocity_trajectory{std::numeric_limits<double>::quiet_NaN()};
};
struct JointCommand {
    double position{0.0}, current_limit{0.0};
};
struct SentGoal {
    bool valid{false};
    double position{0.0};
    int64_t raw{0};
};
// 実行中の関節。設定値と観測・指令は別の領域に保持する。
struct Joint {
    JointConfig config;
    JointState state;
    JointCommand command;
    SentGoal sent;
};
inline std::vector<Joint> make_joints(const std::vector<JointConfig> &configs) {
    std::vector<Joint> joints;
    joints.reserve(configs.size());
    for (const auto &config : configs) {
        Joint joint;
        joint.config = config;
        joint.state.name = config.joint_name;
        if (config.limit_pos_min <= config.limit_pos_max)
            joint.command.position = std::clamp(0.0, config.limit_pos_min, config.limit_pos_max);
        joint.command.current_limit = config.initial_current_limit;
        joints.push_back(std::move(joint));
    }
    return joints;
}
} // namespace fuzzrobo::topoarm_hardware
