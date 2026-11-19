// Copyright 2025 FuzzRoBo CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "dynamixel_communicator.h"
#include "topoarm_hardware/joint_types.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fuzzrobo::topoarm_hardware {
enum class BusError { None, Configuration, Communication, Incomplete, Hardware, InvalidCommand };
struct BusResult {
    BusError error{BusError::None};
    int id{-1};
    std::string detail;
    explicit operator bool() const { return error == BusError::None; }
};
struct ServoFault {
    uint8_t id;
    int64_t error;
};
// lib_dynamixelを維持し、通信境界を注入して実機なしで検証する。
class Transport {
  public:
    virtual ~Transport() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool ping(uint8_t id) = 0;
    // 非対応の通信実装では復旧しない。成功は再起動後のPingまで確認する。
    virtual bool reboot(uint8_t) { return false; }
    virtual bool healthy() = 0;
    virtual bool hardware_error() = 0;
    virtual int64_t read(DynamixelAddress address, uint8_t id) = 0;
    virtual bool write(DynamixelAddress address, uint8_t id, int64_t value) = 0;
    virtual std::map<uint8_t, std::vector<int64_t>> sync_read(const std::vector<DynamixelAddress> &addresses,
                                                              const std::vector<uint8_t> &ids) = 0;
    virtual bool sync_write(DynamixelAddress address, const std::vector<uint8_t> &ids,
                            const std::vector<int64_t> &values) = 0;
};
class DynamixelHandler {
  public:
    std::vector<Joint> dxls;
    std::vector<uint8_t> dxl_ids;
    DynamixelHandler(const std::string &port, int baud, int latency);
    explicit DynamixelHandler(std::unique_ptr<Transport> transport);
    virtual ~DynamixelHandler();
    virtual bool open_port();
    virtual bool close_port();
    virtual BusResult scan(const std::vector<uint8_t> &ids, const std::vector<std::string> &names);
    virtual BusResult set_torque(bool enabled);
    virtual BusResult configure(bool allow_changes);
    bool configure_return_delay_time(int raw);
    bool write_position_pid();
    virtual bool read_all(bool full = true);
    virtual bool set_joints_current();
    bool check_torque_off_target_write();
    virtual bool set_joints_position(bool acknowledged = false);
    virtual bool verify_hold();
    bool set_goal_to_present();
    const std::vector<ServoFault> &faults() const { return faults_; }
    bool recover_faults();
    static int64_t position_goal_pulse(const Joint &joint, double position);
    const BusResult &last_result() const { return last_result_; }

  protected:
    std::unique_ptr<Transport> transport_;
    BusResult last_result_;
    std::vector<ServoFault> faults_;
    bool reset_conditions(uint8_t id, bool expect_clear);
    bool fail(BusError code, int id, const std::string &detail);
    bool write_verified(DynamixelAddress address, uint8_t id, int64_t value,
                        const std::string &context, bool check_alert = true);
    bool configure_joint(Joint &joint, bool allow_changes);
};
} // namespace fuzzrobo::topoarm_hardware
