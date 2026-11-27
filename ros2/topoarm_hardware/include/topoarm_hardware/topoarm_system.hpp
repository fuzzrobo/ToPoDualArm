// Copyright 2025 FuzzRoBo CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topoarm_hardware/dynamixel_handler.hpp"
#include "topoarm_hardware/driver_session.hpp"

namespace fuzzrobo::topoarm_hardware {
struct MimicInfo {
    std::string name;
    double multiplier{1}, offset{0}, position{0}, velocity{0};
    size_t source{0};
};
class ToPoArmSystemHardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo&) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;
protected:
    // テストはこのfactoryへ疑似通信を注入し、シリアルポートを開かない。
    virtual std::unique_ptr<DynamixelHandler> make_handler(const std::string&, int, int);
    std::unique_ptr<DynamixelHandler> handler_;
private:
    std::vector<MimicInfo> mimics_;
    bool verified_{false}, read_only_{false}, vsido_{false};
    std::unique_ptr<JointIO> session_;
    rclcpp::Logger logger_{rclcpp::get_logger("topoarm_hardware")};
    bool leader_torque_off();
    void update_mimics();
};
}  // namespace fuzzrobo::topoarm_hardware
