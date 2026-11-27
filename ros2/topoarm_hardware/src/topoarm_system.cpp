// Copyright 2025 FuzzRoBo CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/topoarm_system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "topoarm_hardware/config.hpp"
#include "topoarm_hardware/vsido.hpp"
#include <initializer_list>
#include <set>
#include <stdexcept>

namespace fuzzrobo::topoarm_hardware {
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace {
bool boolean(const std::string &value) {
    if (value == "true" || value == "1")
        return true;
    if (value == "false" || value == "0")
        return false;
    throw std::invalid_argument("真偽値が不正です");
}
} // namespace
std::unique_ptr<DynamixelHandler> ToPoArmSystemHardware::make_handler(const std::string &port, int baud,
                                                                      int latency) {
    return std::make_unique<DynamixelHandler>(port, baud, latency);
}
CallbackReturn ToPoArmSystemHardware::on_init(const hardware_interface::HardwareInfo &info) {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;
    try {
        const auto &p = info_.hardware_parameters;
        auto get = [&](const std::string &key, const std::string &fallback) {
            const auto it = p.find(key);
            return it == p.end() ? fallback : it->second;
        };
        if (!p.count("config_file"))
            throw std::invalid_argument("SDKのconfig_fileを指定してください");
        // 実機値はYAMLだけで管理し、古いROSパラメータとの二重指定を拒否する。
        for (const auto &key : {"usb_port", "baud_rate", "latency_timer"})
            if (p.count(key))
                throw std::invalid_argument(std::string(key) + "はSDK YAMLへ移してください");
        const auto backend = get("backend", "u2d2");
        if (backend != "u2d2" && backend != "vsido")
            throw std::invalid_argument("backendにはu2d2またはvsidoを指定してください");
        vsido_ = backend == "vsido";
        read_only_ = boolean(get("read_only", "false"));
        const bool torque = boolean(get("torque", "false"));
        const bool configure_devices = boolean(get("configure_devices", "false"));
        const bool verified_requested = boolean(get("hardware_verified", "true"));
        if (read_only_ && (torque || configure_devices))
            throw std::invalid_argument("読取り専用モードでは駆動・機器設定を変更できません");
        if (vsido_) {
            auto config = load_vsido_config(p.at("config_file"), get("vsido_peer_address", ""),
                                            get("vsido_bind_address", ""));
            verified_ = verified_requested && config.hardware_verified;
            config.options.verified = verified_;
            config.options.torque = torque;
            config.options.configure_devices = configure_devices;
            session_ = std::make_unique<VsidoSession>(std::move(config));
        } else {
            auto config = load_config(p.at("config_file"));
            auto options = config.options;
            verified_ = verified_requested && config.hardware_verified;
            options.verified = verified_;
            options.torque = torque;
            options.configure_devices = configure_devices;
            // リーダーでもYAMLで明示した起動設定は適用する。
            if (read_only_ && !options.apply_startup_configuration)
                options.return_delay_time = -1;
            if (options.torque && !options.floor_enabled)
                throw std::invalid_argument("実機フォロワーの駆動には床保護が必要です");
            handler_ = make_handler(config.port, config.baud_rate, config.latency_timer);
            handler_->dxls = make_joints(config.joints);
            for (const auto &d : config.joints)
                handler_->dxl_ids.push_back(d.id);
            session_ = std::make_unique<DriverSession>(*handler_, options);
        }
        auto &joints = session_->joints();
        std::set<std::string> names;
        for (const auto &joint : info_.joints) {
            if (joint.parameters.count("mimic"))
                continue;
            if (!joint.parameters.empty())
                throw std::invalid_argument("関節の実機設定はSDK YAMLへ移してください: " + joint.name);
            if (!names.insert(joint.name).second)
                throw std::invalid_argument("関節名が重複しています");
            const auto it = std::find_if(joints.begin(), joints.end(),
                                         [&](const auto &d) { return d.config.joint_name == joint.name; });
            if (it == joints.end())
                throw std::invalid_argument("YAMLとROSの関節名が不一致です: " + joint.name);
            if (read_only_) {
                if (!joint.command_interfaces.empty())
                    throw std::invalid_argument("読取り専用の関節にcommand interfaceがあります");
                continue;
            }
            if (joint.command_interfaces.size() != 1 || joint.command_interfaces.front().name != "position")
                throw std::invalid_argument("command interfaceにはpositionを指定してください: " + joint.name);
        }
        if (names.size() != joints.size())
            throw std::invalid_argument("YAMLとROSの全関節が一致する必要があります");
        for (const auto &joint : info_.joints) {
            if (!joint.parameters.count("mimic"))
                continue;
            const auto &source = joint.parameters.at("mimic");
            const auto it = std::find_if(joints.begin(), joints.end(),
                                         [&](const auto &d) { return d.config.joint_name == source; });
            if (it == joints.end() || !joint.command_interfaces.empty())
                throw std::invalid_argument("mimicの接続またはcommand interfaceが不正です");
            MimicInfo m;
            m.name = joint.name;
            m.source = std::distance(joints.begin(), it);
            if (joint.parameters.count("multiplier"))
                m.multiplier = std::stod(joint.parameters.at("multiplier"));
            if (joint.parameters.count("offset"))
                m.offset = std::stod(joint.parameters.at("offset"));
            if (!std::isfinite(m.multiplier) || !std::isfinite(m.offset))
                throw std::invalid_argument("mimic変換が不正です");
            mimics_.push_back(m);
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "初期化失敗: %s", e.what());
        return CallbackReturn::ERROR;
    }
    // on_initでは通信・EEPROM書込み・Torque操作をしない。
    return CallbackReturn::SUCCESS;
}
CallbackReturn ToPoArmSystemHardware::on_configure(const rclcpp_lifecycle::State &) {
    if (!verified_) {
        RCLCPP_ERROR(logger_, "機体YAMLまたは起動引数でhardware_verifiedが無効です");
        return CallbackReturn::FAILURE;
    }
    if (!session_->configure()) {
        RCLCPP_ERROR(logger_, "%s", session_->error().c_str());
        return CallbackReturn::ERROR;
    }
    if (!leader_torque_off())
        return CallbackReturn::ERROR;
    update_mimics();
    return CallbackReturn::SUCCESS;
}
CallbackReturn ToPoArmSystemHardware::on_activate(const rclcpp_lifecycle::State &) {
    return session_->activate() && leader_torque_off() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}
CallbackReturn ToPoArmSystemHardware::on_deactivate(const rclcpp_lifecycle::State &) {
    return session_->deactivate() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}
CallbackReturn ToPoArmSystemHardware::on_cleanup(const rclcpp_lifecycle::State &) {
    session_->close();
    return CallbackReturn::SUCCESS;
}
CallbackReturn ToPoArmSystemHardware::on_error(const rclcpp_lifecycle::State &) {
    // SDKで復旧できなかった異常はROSのエラー処理で接続を終了する。
    session_->abort();
    return CallbackReturn::FAILURE;
}
std::vector<hardware_interface::StateInterface> ToPoArmSystemHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> out;
    struct Field {
        const char *name;
        double *value;
        bool u2d2_only{false};
    };
    for (auto &d : session_->joints()) {
        for (const auto &[name, value, u2d2_only] : std::initializer_list<Field>{
                 {"position", &d.state.position},
                 {"velocity", &d.state.velocity},
                 {"current", &d.state.current},
                 {"pwm", &d.state.pwm, true},
                 {"voltage", &d.state.voltage, true},
                 {"temperature", &d.state.temperature, true},
                 {"valid", &d.state.valid},
                 {"age", &d.state.age},
                 {"torque_enabled", &d.state.torque_enabled, true},
                 {"hardware_error", &d.state.hardware_error, true},
                 {"realtime_tick", &d.state.realtime_tick, true},
                 {"goal_position_readback", &d.state.goal_position_readback},
                 {"position_command", &d.command.position},
                 {"current_limit", &d.command.current_limit, true}})
            if (!u2d2_only || !vsido_)
                out.emplace_back(d.config.joint_name, name, value);
    }
    for (auto &m : mimics_) {
        out.emplace_back(m.name, "position", &m.position);
        out.emplace_back(m.name, "velocity", &m.velocity);
    }
    out.emplace_back("bus", "read_start_monotonic_s", &session_->read_start);
    out.emplace_back("bus", "read_end_monotonic_s", &session_->read_end);
    out.emplace_back("bus", "write_start_monotonic_s", &session_->write_start);
    out.emplace_back("bus", "write_end_monotonic_s", &session_->write_end);
    out.emplace_back("bus", "write_ok", &session_->write_ok);
    return out;
}
std::vector<hardware_interface::CommandInterface> ToPoArmSystemHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> out;
    if (read_only_)
        return out;
    for (auto &d : session_->joints())
        out.emplace_back(d.config.joint_name, "position", &d.command.position);
    return out;
}
bool ToPoArmSystemHardware::leader_torque_off() {
    if (!read_only_ || vsido_)
        return true;
    for (const auto &joint : session_->joints()) {
        if (joint.state.torque_enabled != 0) {
            RCLCPP_ERROR(logger_, "リーダーのTorque ONを検出しました: ID=%d", joint.config.id);
            session_->abort();
            return false;
        }
    }
    return true;
}
void ToPoArmSystemHardware::update_mimics() {
    for (auto &m : mimics_) {
        const auto &d = session_->joints()[m.source];
        m.position = m.multiplier * d.state.position + m.offset;
        m.velocity = m.multiplier * d.state.velocity;
    }
}
return_type ToPoArmSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
    if (!session_->read()) {
        RCLCPP_ERROR(logger_, "%s", session_->error().c_str());
        return return_type::ERROR;
    }
    if (!leader_torque_off())
        return return_type::ERROR;
    if (session_->fresh())
        update_mimics();
    return return_type::OK;
}
return_type ToPoArmSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
    if (read_only_)
        return return_type::OK;
    if (!session_->write()) {
        RCLCPP_ERROR(logger_, "%s", session_->error().c_str());
        return return_type::ERROR;
    }
    return return_type::OK;
}
} // namespace fuzzrobo::topoarm_hardware
PLUGINLIB_EXPORT_CLASS(fuzzrobo::topoarm_hardware::ToPoArmSystemHardware, hardware_interface::SystemInterface)
