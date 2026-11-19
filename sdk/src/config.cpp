// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/config.hpp"
#include <filesystem>
#include <set>
#include <yaml-cpp/yaml.h>
namespace fuzzrobo::topoarm_hardware {
namespace {
void keys(const YAML::Node &node, const std::set<std::string> &allowed, const std::string &context) {
    if (!node.IsMap())
        throw std::invalid_argument(context + "はマッピングが必要です");
    std::set<std::string> seen;
    for (const auto &item : node) {
        const auto key = item.first.as<std::string>();
        if (!seen.insert(key).second)
            throw std::invalid_argument(context + ": 重複キー " + key);
        if (!allowed.empty() && !allowed.count(key))
            throw std::invalid_argument(context + ": 未対応キー " + key);
    }
}
template <class T> void read(const YAML::Node &node, const char *key, T &value) {
    if (node[key])
        value = node[key].as<T>();
}
} // namespace
HardwareConfig load_config(const std::string &path, const std::vector<std::string> &selected) {
    try {
        const auto root = YAML::LoadFile(path);
        keys(root,
             {"hardware_verified", "connection", "options", "joints", "calibration_note",
              "motor_settings_note", "range_note", "neck_calibration_note", "gripper_mapping_note",
              "input_transform", "model", "actuator_profiles"},
             "設定");
        HardwareConfig config;
        read(root, "hardware_verified", config.hardware_verified);
        if (auto c = root["connection"]) {
            keys(c, {"port", "baud_rate", "latency_timer"}, "connection");
            read(c, "port", config.port);
            read(c, "baud_rate", config.baud_rate);
            read(c, "latency_timer", config.latency_timer);
        }
        if (config.port.empty() || config.baud_rate <= 0 || config.latency_timer < 1 ||
            config.latency_timer > 255)
            throw std::invalid_argument("接続設定が不正です");
        if (auto o = root["options"]) {
            keys(o,
                 {"torque_retention", "clip_to_limits", "write_pid_on_connect", "return_delay_time", "auto_reset",
                  "neck_envelope", "neck_limits", "floor_config"},
                 "options");
            read(o, "torque_retention", config.options.retention);
            read(o, "clip_to_limits", config.options.clip_to_limits);
            read(o, "write_pid_on_connect", config.options.write_pid_on_connect);
            read(o, "return_delay_time", config.options.return_delay_time);
            read(o, "auto_reset", config.options.auto_reset);
            read(o, "neck_envelope", config.options.neck_envelope);
            if (auto n = o["neck_limits"]) {
                keys(n, {"pan_abs_at_outer", "tilt_min_at_outer", "outward_margin"}, "neck_limits");
                auto &limits = config.options.neck_limits;
                limits.pan_abs_at_outer = n["pan_abs_at_outer"].as<double>();
                limits.tilt_min_at_outer = n["tilt_min_at_outer"].as<double>();
                limits.outward_margin = n["outward_margin"].as<double>();
            }
            if (auto f = o["floor_config"]) {
                auto floor_path = std::filesystem::path(f.as<std::string>());
                if (floor_path.empty())
                    throw std::invalid_argument("floor_configが空です");
                if (floor_path.is_relative())
                    floor_path = std::filesystem::absolute(path).parent_path() / floor_path;
                const auto floor = YAML::LoadFile(floor_path.lexically_normal().string());
                keys(floor, {"enabled", "z_m", "margin_m"}, "floor");
                config.options.floor_enabled = floor["enabled"].as<bool>();
                config.options.floor_z_m = floor["z_m"].as<double>();
                config.options.floor_margin_m = floor["margin_m"].as<double>();
            }
        }
        if (auto value = root["model"]) {
            auto model = std::filesystem::path(value.as<std::string>());
            if (model.empty())
                throw std::invalid_argument("modelが空です");
            if (model.is_relative())
                model = std::filesystem::absolute(path).parent_path() / model;
            config.options.robot_model = model.lexically_normal().string();
        }
        if (auto t = root["input_transform"]) {
            keys(t, {"neck_tilt_negative_scale"}, "input_transform");
            read(t, "neck_tilt_negative_scale", config.neck_tilt_negative_scale);
            if (!std::isfinite(config.neck_tilt_negative_scale) || config.neck_tilt_negative_scale <= 0)
                throw std::invalid_argument("下側の角度倍率は有限の正数が必要です");
        }
        YAML::Node profiles;
        if (const auto source = root["actuator_profiles"]) {
            const auto file = std::filesystem::absolute(path).parent_path() / source.as<std::string>();
            profiles = YAML::LoadFile(file.lexically_normal().string());
            keys(profiles, {}, "actuator_profiles");
            for (const auto &preset : profiles)
                keys(preset.second, {"position_kp", "position_ki", "position_kd", "profile_velocity",
                     "profile_acceleration", "profile_duration_ms", "profile_acceleration_ms"}, "servo profile");
        }
        const auto joints = root["joints"];
        keys(joints, {}, "joints");
        for (const auto &item : joints) {
            JointConfig d;
            d.joint_name = item.first.as<std::string>();
            keys(item.second, {}, d.joint_name);
            auto j = YAML::Clone(item.second);
            if (const auto p = item.second["profile"]) {
                const auto preset = profiles[p.as<std::string>()];
                if (!preset.IsMap())
                    throw std::invalid_argument("未定義のサーボprofile: " + p.as<std::string>());
                for (const auto &entry : preset) {
                    const auto key = entry.first.as<std::string>();
                    if (!j[key]) j[key] = YAML::Clone(entry.second);
                }
                j.remove("profile");
            }
            keys(j,
                 {"id", "operating_mode", "multiplier", "offset", "min", "max", "current_cap",
                  "initial_current_limit", "position_kp", "position_ki", "position_kd", "profile_velocity",
                  "profile_acceleration", "profile_duration_ms", "profile_acceleration_ms"},
                 d.joint_name);
            const int id = j["id"].as<int>(), mode = j["operating_mode"].as<int>();
            if (id < 1 || id > 253 || (mode != 3 && mode != 5))
                throw std::invalid_argument("IDまたはmodeが不正です: " + d.joint_name);
            d.id = id;
            d.operating_mode = mode;
            d.multiplier = j["multiplier"].as<double>();
            d.offset = j["offset"].as<double>();
            d.limit_pos_min = j["min"].as<double>();
            d.limit_pos_max = j["max"].as<double>();
            d.current_cap = j["current_cap"].as<double>();
            d.initial_current_limit = j["initial_current_limit"].as<double>();
            d.position_kp = j["position_kp"].as<int>();
            d.position_ki = j["position_ki"].as<int>();
            d.position_kd = j["position_kd"].as<int>();
            d.profile_vel = j["profile_velocity"].as<double>();
            d.profile_acc = j["profile_acceleration"].as<double>();
            read(j, "profile_duration_ms", d.profile_duration_ms);
            read(j, "profile_acceleration_ms", d.profile_acceleration_ms);
            config.joints.push_back(d);
        }
        DriverSession::validate(make_joints(config.joints), config.options);
        if (!selected.empty()) {
            std::vector<JointConfig> result;
            std::set<std::string> seen;
            for (const auto &name : selected) {
                if (!seen.insert(name).second)
                    throw std::invalid_argument("対象関節が重複しています: " + name);
                const auto it = std::find_if(config.joints.begin(), config.joints.end(),
                                             [&](const auto &d) { return d.joint_name == name; });
                if (it == config.joints.end())
                    throw std::invalid_argument("設定に関節がありません: " + name);
                result.push_back(*it);
            }
            config.joints = std::move(result);
        }
        return config;
    } catch (const YAML::Exception &e) {
        throw std::invalid_argument("YAML設定を読み込めません: " + path + ": " + e.what());
    }
}
} // namespace fuzzrobo::topoarm_hardware
