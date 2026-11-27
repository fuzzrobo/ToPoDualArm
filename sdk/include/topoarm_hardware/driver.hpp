// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/config.hpp"
#include <mutex>
namespace fuzzrobo::topoarm_hardware {
// C++/Pythonで共有する、同期アクセス付きの接続API。
class Driver {

    DynamixelHandler handler_;
    std::unique_ptr<DriverSession> session_;
    SessionOptions options_;
    std::mutex mutex_;
    void initialize(const std::vector<JointConfig> &joints);

  public:
    Driver(const std::string &port, int baud, int latency, const std::vector<JointConfig> &joints,
           SessionOptions options);
    Driver(std::unique_ptr<Transport> transport, const std::vector<JointConfig> &joints,
           SessionOptions options);
    ~Driver();
    void connect();
    void open_passive();
    void enable();
    void disable();
    void close();
    std::vector<JointState> read();
    size_t reset_count();
    std::vector<uint8_t> last_reset_ids();
    bool reset_pending();
    void resume_after_reset();
    void command_torque_off(const std::vector<double> &positions);
    std::string verify_targets_torque_off();
    std::vector<double> prepare_floor_targets(const std::vector<double> &positions);
    FloorGuard::Result floor_result();
    std::vector<double> command(const std::vector<double> &positions);
};
} // namespace fuzzrobo::topoarm_hardware
