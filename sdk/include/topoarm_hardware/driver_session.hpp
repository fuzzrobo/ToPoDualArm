// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/dynamixel_handler.hpp"
#include "topoarm_hardware/floor_guard.hpp"
#include "topoarm_hardware/session_options.hpp"
#include "topoarm_hardware/target_limiter.hpp"
#include <chrono>
#include <set>

namespace fuzzrobo::topoarm_hardware {
// 接続と明示操作を共有する。エラー後の継続・終了は呼び出し側が決める。
class DriverSession {
  public:
    DriverSession(DynamixelHandler &handler, SessionOptions options);
    bool configure(bool passive = false);
    bool enable_passive();
    bool activate();
    bool deactivate();
    void close();
    void abort();
    bool read();
    bool write();
    // 同じ新しい実測値に対して、送信なしで床補正だけを評価できる。
    bool prepare_floor_targets();
    const FloorGuard::Result &floor_result() const { return limiter_->floor_result(); }
    // 明示的な診断専用。毎回全軸のTorque OFFを読んでから目標レジスタだけ更新する。
    bool write_torque_off();
    bool fresh() const { return fresh_; }
    bool active() const { return active_; }
    const std::string &error() const { return error_; }
    size_t reset_count() const { return reset_count_; }
    const std::vector<uint8_t> &last_reset_ids() const { return last_reset_ids_; }
    bool reset_pending() const { return reset_pending_; }
    bool resume_after_reset();
    double read_start{0}, read_end{0}, write_start{0}, write_end{0}, write_ok{0};
    static void validate(const std::vector<Joint> &joints, const SessionOptions &options);

  private:
    DynamixelHandler &handler_;
    SessionOptions options_;
    std::unique_ptr<TargetLimiter> limiter_;
    bool connected_{false}, active_{false}, fresh_{false};
    std::string error_;
    std::set<uint8_t> reset_attempted_;
    std::vector<uint8_t> last_reset_ids_;
    size_t reset_count_{0};
    bool reset_pending_{false};
    bool fail(const std::string &reason);
    bool positions_valid() const;
    bool write_impl(bool torque_off_diagnostic);
};
} // namespace fuzzrobo::topoarm_hardware
