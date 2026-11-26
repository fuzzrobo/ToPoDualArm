// Copyright 2025 FuzzRoBo CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/topoarm_system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "topoarm_hardware/config.hpp"
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
        auto config = load_config(p.at("config_file"));
        auto options = config.options;
        read_only_ = boolean(get("read_only", "false"));
        options.torque = boolean(get("torque", "false"));
        verified_ = boolean(get("hardware_verified", "false")) && config.hardware_verified;
        options.configure_devices = boolean(get("configure_devices", "false"));
        if (read_only_) {
            if (options.torque || options.configure_devices)
                throw std::invalid_argument("リーダーの読取り専用モードではTorque・機器設定を変更できません");
            // リーダーでもYAMLで明示した起動設定は適用する。
            if (!options.apply_startup_configuration)
                options.return_delay_time = -1;
        }
        options.verified = verified_;
        if (options.torque && !options.floor_enabled)
            throw std::invalid_argument("実機フォロワーの駆動には床保護が必要です");
        std::set<std::string> names;
        for (const auto &joint : info_.joints) {
            if (joint.parameters.count("mimic"))
                continue;
            if (!joint.parameters.empty())
                throw std::invalid_argument("関節の実機設定はSDK YAMLへ移してください: " + joint.name);
            if (!names.insert(joint.name).second)
                throw std::invalid_argument("関節名が重複しています");
            const auto it = std::find_if(config.joints.begin(), config.joints.end(),
                                         [&](const auto &d) { return d.joint_name == joint.name; });
            if (it == config.joints.end())
                throw std::invalid_argument("YAMLとROSの関節名が不一致です: " + joint.name);
            if (read_only_) {
                if (!joint.command_interfaces.empty())
                    throw std::invalid_argument("読取り専用の関節にcommand interfaceがあります");
                continue;
            }
            bool position = false;
            for (const auto &command : joint.command_interfaces) {
                if (command.name == "position")
                    position = true;
                else if (command.name != "current_limit" || it->operating_mode != 5)
                    throw std::invalid_argument("未対応のcommand interfaceです");
            }
            if (!position)
                throw std::invalid_argument("position interfaceが必要です");
        }
        if (names.size() != config.joints.size())
            throw std::invalid_argument("YAMLとROSの全関節が一致する必要があります");
        handler_ = make_handler(config.port, config.baud_rate, config.latency_timer);
        handler_->dxls = make_joints(config.joints);
        for (const auto &d : config.joints)
            handler_->dxl_ids.push_back(d.id);
        for (const auto &joint : info_.joints) {
            if (!joint.parameters.count("mimic"))
                continue;
            const auto &source = joint.parameters.at("mimic");
            const auto it = std::find_if(handler_->dxls.begin(), handler_->dxls.end(),
                                         [&](const auto &d) { return d.config.joint_name == source; });
            if (it == handler_->dxls.end() || !joint.command_interfaces.empty())
                throw std::invalid_argument("mimicの接続またはcommand interfaceが不正です");
            MimicInfo m;
            m.name = joint.name;
            m.source = std::distance(handler_->dxls.begin(), it);
            if (joint.parameters.count("multiplier"))
                m.multiplier = std::stod(joint.parameters.at("multiplier"));
            if (joint.parameters.count("offset"))
                m.offset = std::stod(joint.parameters.at("offset"));
            if (!std::isfinite(m.multiplier) || !std::isfinite(m.offset))
                throw std::invalid_argument("mimic変換が不正です");
            mimics_.push_back(m);
        }
        session_ = std::make_unique<DriverSession>(*handler_, options);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "初期化失敗: %s", e.what());
        return CallbackReturn::ERROR;
    }
    // on_initでは通信・EEPROM書込み・Torque操作をしない。
    return CallbackReturn::SUCCESS;
}
CallbackReturn ToPoArmSystemHardware::on_configure(const rclcpp_lifecycle::State &) {
    if (!verified_) {
        RCLCPP_ERROR(logger_, "実機校正・可動域の確認後にhardware_verifiedを明示してください");
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
    for (auto &d : handler_->dxls) {
        for (auto [name, value] : std::vector<std::pair<std::string, double *>>{
                 {"position", &d.state.position},
                 {"velocity", &d.state.velocity},
                 {"current", &d.state.current},
                 {"pwm", &d.state.pwm},
                 {"voltage", &d.state.voltage},
                 {"temperature", &d.state.temperature},
                 {"valid", &d.state.valid},
                 {"age", &d.state.age},
                 {"torque_enabled", &d.state.torque_enabled},
                 {"hardware_error", &d.state.hardware_error},
                 {"realtime_tick", &d.state.realtime_tick},
                 {"goal_position_readback", &d.state.goal_position_readback},
                 {"position_command", &d.command.position},
                 {"current_limit", &d.command.current_limit}})
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
    for (auto &d : handler_->dxls) {
        out.emplace_back(d.config.joint_name, "position", &d.command.position);
        if (d.config.operating_mode == 5)
            out.emplace_back(d.config.joint_name, "current_limit", &d.command.current_limit);
    }
    return out;
}
bool ToPoArmSystemHardware::leader_torque_off() {
    if (!read_only_)
        return true;
    for (const auto &joint : handler_->dxls) {
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
        const auto &d = handler_->dxls[m.source];
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
