// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/config.hpp"
#include "topoarm_hardware/vsido.hpp"
#include <filesystem>
#include <set>
#include <yaml-cpp/yaml.h>
namespace fuzzrobo::topoarm_hardware {
namespace {
const std::set<std::string> hardware_keys{
    "hardware_verified", "connection", "options", "joints", "calibration_note",
    "motor_settings_note", "range_note", "neck_calibration_note", "gripper_mapping_note",
    "input_transform", "model", "modules", "pan_tilt", "actuator_profiles", "vsido"};
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
constexpr auto neck_pan = "neck_pan_joint";
constexpr auto neck_tilt = "neck_tilt_joint";

bool uses_pan_tilt(const YAML::Node &root) {
    const auto modules = root["modules"];
    if (!modules)
        return false;
    if (!root["model"] || root["model"].as<std::string>() != "300mm")
        throw std::invalid_argument("modulesにはmodel: 300mmを指定してください");
    if (!modules.IsSequence() || modules.size() > 1 ||
        (modules.size() == 1 && modules[0].as<std::string>() != "pan_tilt"))
        throw std::invalid_argument("modulesには[]または[pan_tilt]を指定してください");
    return modules.size() == 1;
}

YAML::Node read_joints(const YAML::Node &root, bool pan_tilt) {
    const auto base = root["joints"];
    keys(base, {}, "joints");
    if (root["model"] && root["model"].as<std::string>() == "300mm" &&
        (base[neck_pan] || base[neck_tilt]))
        throw std::invalid_argument("首の関節はpan_tilt.jointsに指定してください");
    auto joints = YAML::Clone(base);
    if (pan_tilt) {
        const auto module = root["pan_tilt"];
        keys(module, {"joints"}, "pan_tilt");
        const auto extra = module["joints"];
        keys(extra, {neck_pan, neck_tilt}, "pan_tilt.joints");
        if (extra.size() != 2)
            throw std::invalid_argument("pan_tilt.jointsにはpan/tilt両軸が必要です");
        for (const auto &axis : extra)
            joints[axis.first.as<std::string>()] = YAML::Clone(axis.second);
    }
    return joints;
}

void read_geometry(const YAML::Node &root, const std::string &path, bool pan_tilt, SessionOptions &options) {
    const auto directory = std::filesystem::absolute(path).parent_path();
    const auto model = root["model"];
    const bool named_model = model && model.as<std::string>() == "300mm";
    if (auto o = root["options"]) {
        read(o, "clip_to_limits", options.clip_to_limits);
        if (!named_model || pan_tilt) {
            read(o, "neck_envelope", options.neck_envelope);
            if (auto n = o["neck_limits"]) {
                keys(n, {"pan_abs_at_outer", "tilt_min_at_outer", "outward_margin"}, "neck_limits");
                options.neck_limits.pan_abs_at_outer = n["pan_abs_at_outer"].as<double>();
                options.neck_limits.tilt_min_at_outer = n["tilt_min_at_outer"].as<double>();
                options.neck_limits.outward_margin = n["outward_margin"].as<double>();
            }
        }
        if (auto f = o["floor_config"]) {
            auto file = std::filesystem::path(f.as<std::string>());
            if (file.empty())
                throw std::invalid_argument("floor_configが空です");
            const auto floor = YAML::LoadFile((directory / file).lexically_normal().string());
            keys(floor, {"enabled", "z_m", "margin_m"}, "floor");
            options.floor_enabled = floor["enabled"].as<bool>();
            options.floor_z_m = floor["z_m"].as<double>();
            options.floor_margin_m = floor["margin_m"].as<double>();
        }
    }
    if (model) {
        const auto models = (directory / "../../models/300mm").lexically_normal();
        const auto standard = models / "robot.urdf";
        const auto model_value = model.as<std::string>();
        if (model_value.empty())
            throw std::invalid_argument("modelが空です");
        auto file = (directory / model_value).lexically_normal();
        const auto axes = root["joints"];
        const bool legacy_equipped = !named_model && axes.IsMap() && axes[neck_pan] && axes[neck_tilt] &&
                                     (file == standard || file == models / "robot_pan_tilt.urdf");
        if (named_model || legacy_equipped)
            file = standard;
        options.robot_model = file.string();
        if (pan_tilt || legacy_equipped)
            options.robot_model_fragment = (models / "pan_tilt.urdf").string();
    }
}

void read_axis(const YAML::Node &node, JointConfig &joint) {
    const int id = node["id"].as<int>();
    if (id < 1 || id > 253)
        throw std::invalid_argument("IDが不正です: " + joint.joint_name);
    joint.id = id;
    joint.multiplier = node["multiplier"].as<double>();
    joint.offset = node["offset"].as<double>();
    joint.limit_pos_min = node["min"].as<double>();
    joint.limit_pos_max = node["max"].as<double>();
}
} // namespace
HardwareConfig load_config(const std::string &path, const std::vector<std::string> &selected) {
    try {
        const auto root = YAML::LoadFile(path);
        keys(root, hardware_keys, "設定");
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
                  "neck_envelope", "neck_limits", "floor_config", "read_profile", "startup_configuration"},
                 "options");
            read(o, "torque_retention", config.options.retention);
            read(o, "write_pid_on_connect", config.options.write_pid_on_connect);
            read(o, "return_delay_time", config.options.return_delay_time);
            read(o, "read_profile", config.options.read_profile);
            if (auto value = o["startup_configuration"]) {
                const auto policy = value.as<std::string>();
                if (policy != "verify" && policy != "apply")
                    throw std::invalid_argument("startup_configurationはverifyまたはapplyを指定してください");
                config.options.apply_startup_configuration = policy == "apply";
            }
            // 旧YAMLとの互換用。値の型だけ検査し、自動復旧は常時有効とする。
            if (o["auto_reset"])
                (void)o["auto_reset"].as<bool>();
        }
        const bool pan_tilt = uses_pan_tilt(root);
        read_geometry(root, path, pan_tilt, config.options);
        if (auto t = root["input_transform"]) {
            keys(t, {"neck_tilt_negative_scale"}, "input_transform");
            read(t, "neck_tilt_negative_scale", config.neck_tilt_negative_scale);
            if (!std::isfinite(config.neck_tilt_negative_scale) || config.neck_tilt_negative_scale <= 0)
                throw std::invalid_argument("下側の角度倍率は有限の正数が必要です");
        }
        YAML::Node profiles;
        if (const auto source = root["actuator_profiles"]) {
            if (source.IsMap()) {
                profiles = source;
            } else {
                const auto file = std::filesystem::absolute(path).parent_path() / source.as<std::string>();
                profiles = YAML::LoadFile(file.lexically_normal().string());
            }
            keys(profiles, {}, "actuator_profiles");
            for (const auto &preset : profiles)
                keys(preset.second, {"position_kp", "position_ki", "position_kd", "profile_velocity",
                     "profile_acceleration", "profile_duration_ms", "profile_acceleration_ms"}, "servo profile");
        }
        const auto joints = read_joints(root, pan_tilt);
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
            read_axis(j, d);
            const int mode = j["operating_mode"].as<int>();
            if (mode != 3 && mode != 5)
                throw std::invalid_argument("modeが不正です: " + d.joint_name);
            d.operating_mode = mode;
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

VsidoConfig load_vsido_config(const std::string &path, const std::string &peer_address,
                              const std::string &bind_address) {
    try {
        const auto root = YAML::LoadFile(path);
        const auto network = root["vsido"];
        if (network)
            keys(root, hardware_keys, "V-Sido設定");
        else
            keys(root, {"hardware_verified", "connection", "model", "options", "joints", "command_order"},
                 "V-Sido設定");
        VsidoConfig config;
        read(root, "hardware_verified", config.hardware_verified);
        // 旧形式ではUDP設定がconnectionに、送信順がルートにある。
        const auto c = network ? network : root["connection"];
        keys(c, {"bind_address", "peer_address", "receive_port", "command_port", "timeout_s",
                 "command_order"}, "V-Sido接続");
        read(c, "peer_address", config.peer_address);
        read(c, "bind_address", config.bind_address);
        read(c, "receive_port", config.receive_port);
        read(c, "command_port", config.command_port);
        read(c, "timeout_s", config.timeout_s);
        if (!peer_address.empty())
            config.peer_address = peer_address;
        if (!bind_address.empty())
            config.bind_address = bind_address;
        if (config.peer_address.empty())
            throw std::invalid_argument("V-Sidoのpeer_addressを指定してください");
        if (const auto o = root["options"]; o && !network)
            keys(o, {"clip_to_limits", "neck_envelope", "neck_limits", "floor_config"}, "options");
        const bool pan_tilt = uses_pan_tilt(root);
        read_geometry(root, path, pan_tilt, config.options);
        const auto joints = read_joints(root, pan_tilt);
        for (const auto &item : joints) {
            JointConfig joint;
            joint.joint_name = item.first.as<std::string>();
            if (network)
                keys(item.second, {}, joint.joint_name);
            else
                keys(item.second, {"id", "multiplier", "offset", "min", "max"}, joint.joint_name);
            read_axis(item.second, joint);
            joint.operating_mode = 3;
            config.joints.push_back(joint);
        }
        const auto order = network ? network["command_order"] : root["command_order"];
        config.command_order = order.as<std::vector<std::string>>();
        return config;
    } catch (const YAML::Exception &e) {
        throw std::invalid_argument("V-Sido YAML設定を読み込めません: " + path + ": " + e.what());
    }
}
} // namespace fuzzrobo::topoarm_hardware
