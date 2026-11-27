// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/floor_guard.hpp"
#include "topoarm_hardware/joint_types.hpp"
#include "topoarm_hardware/neck_guard.hpp"
#include "topoarm_hardware/session_options.hpp"
#include <memory>
namespace fuzzrobo::topoarm_hardware {
// 通信を行わない目標補正。履歴は送信成功時のcommitでだけ進める。
class TargetLimiter {
  public:
    using Quantize = double (*)(const Joint &, double);
    TargetLimiter(const std::vector<Joint> &joints, const SessionOptions &options, Quantize quantize);
    void reset(const std::vector<Joint> &joints);
    void clear();
    void prepare(std::vector<Joint> &joints);
    void commit(const std::vector<Joint> &joints);
    const FloorGuard::Result &floor_result() const { return floor_result_; }

  private:
    SessionOptions options_;
    Quantize quantize_;
    NeckLimits neck_limits_{};
    std::unique_ptr<FloorGuard> floor_;
    std::vector<size_t> floor_indices_;
    FloorGuard::Result floor_result_;
    std::vector<double> floor_previous_goal_;
    std::array<size_t, 2> neck_indices_{};
    std::array<double, 2> neck_previous_goal_{};
    void prepare_neck_targets(std::vector<Joint> &joints);
};
} // namespace fuzzrobo::topoarm_hardware
