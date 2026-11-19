// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/driver.hpp"
namespace fuzzrobo::topoarm_hardware {
Driver::Driver(const std::string &port, int baud, int latency, const std::vector<JointConfig> &joints,
               SessionOptions options)
    : handler_(port, baud, latency), options_(options) {
    initialize(joints);
}
Driver::Driver(std::unique_ptr<Transport> transport, const std::vector<JointConfig> &joints,
               SessionOptions options)
    : handler_(std::move(transport)), options_(options) {
    initialize(joints);
}
void Driver::initialize(const std::vector<JointConfig> &joints) {
    handler_.dxls = make_joints(joints);
    for (const auto &d : joints)
        handler_.dxl_ids.push_back(d.id);
    session_ = std::make_unique<DriverSession>(handler_, options_);
}
Driver::~Driver() {
    try {
        close();
    } catch (...) {
    }
}
void Driver::connect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!session_->configure())
        throw std::runtime_error(session_->error());
    if (!session_->activate()) {
        session_->close();
        throw std::runtime_error(session_->error());
    }
}
void Driver::open_passive() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!session_->configure(true))
        throw std::runtime_error(session_->error());
}
void Driver::enable() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!options_.torque || !session_->enable_passive())
        throw std::runtime_error("駆動を開始できません: " + session_->error());
}
void Driver::disable() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!session_->deactivate())
        throw std::runtime_error(session_->error());
}
void Driver::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool ok = session_->deactivate();
    session_->close();
    if (!ok)
        throw std::runtime_error(session_->error());
}
std::vector<JointState> Driver::read() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!session_->read() || !session_->fresh())
        throw std::runtime_error("有効な新しい状態を取得できません: " + session_->error());
    std::vector<JointState> states;
    for (const auto &joint : handler_.dxls) {
        auto state = joint.state;
        state.name = joint.config.joint_name;
        states.push_back(std::move(state));
    }
    return states;
}
size_t Driver::reset_count() {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_->reset_count();
}
std::vector<uint8_t> Driver::last_reset_ids() {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_->last_reset_ids();
}
bool Driver::reset_pending() {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_->reset_pending();
}
void Driver::resume_after_reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!session_->resume_after_reset())
        throw std::runtime_error(session_->error());
}
void Driver::command_torque_off(const std::vector<double> &positions) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (options_.torque || options_.configure_devices)
        throw std::runtime_error("Torque OFF診断用接続ではありません");
    if (positions.size() != handler_.dxls.size())
        throw std::invalid_argument("指令の軸数が不一致です");
    for (size_t i = 0; i < positions.size(); ++i)
        handler_.dxls[i].command.position = positions[i];
    if (!session_->write_torque_off() || session_->write_ok != 1)
        throw std::runtime_error("Torque OFF診断の指令は送信されませんでした: " + session_->error());
}
std::string Driver::verify_targets_torque_off() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (options_.torque)
        throw std::runtime_error("Torque OFF診断用接続ではありません");
    if (!session_->read())
        throw std::runtime_error(session_->error());
    for (const auto &d : handler_.dxls)
        if (d.state.torque_enabled != 0)
            throw std::runtime_error("Torque ONを検出しました");
    return handler_.verify_hold()
               ? ""
               : "ID=" + std::to_string(handler_.last_result().id) + " " + handler_.last_result().detail;
}
std::vector<double> Driver::prepare_floor_targets(const std::vector<double> &positions) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (positions.size() != handler_.dxls.size())
        throw std::invalid_argument("床補正の軸数が不一致です");
    for (size_t i = 0; i < positions.size(); ++i)
        handler_.dxls[i].command.position = positions[i];
    if (!session_->prepare_floor_targets())
        throw std::runtime_error(session_->error());
    std::vector<double> out;
    for (const auto &d : handler_.dxls)
        out.push_back(d.command.position);
    return out;
}
FloorGuard::Result Driver::floor_result() {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_->floor_result();
}
std::vector<double> Driver::command(const std::vector<double> &positions) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!options_.torque)
        throw std::runtime_error("torque=falseでは位置指令を送れません");
    if (positions.size() != handler_.dxls.size())
        throw std::invalid_argument("指令の軸数が不一致です");
    for (size_t i = 0; i < positions.size(); ++i)
        handler_.dxls[i].command.position = positions[i];
    if (!session_->write() || session_->write_ok != 1)
        throw std::runtime_error("指令は送信されませんでした: " + session_->error());
    std::vector<double> out;
    for (const auto &d : handler_.dxls)
        out.push_back(d.command.position);
    return out;
}
} // namespace fuzzrobo::topoarm_hardware
