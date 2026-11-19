// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/joint_types.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace fuzzrobo::topoarm_hardware {
// 全角度はrad。固有値はYAMLで指定し、静的上下限は関節設定を共用する。
struct NeckEnvelopeConfig {
    double pan_abs_at_outer{std::numeric_limits<double>::quiet_NaN()};
    double tilt_min_at_outer{std::numeric_limits<double>::quiet_NaN()};
    double outward_margin{std::numeric_limits<double>::quiet_NaN()};
};
struct NeckLimits {
    double pan_min, pan_max, tilt_min, tilt_max;
    NeckEnvelopeConfig envelope;
};
class NeckGuard {
  public:
    static constexpr double rad = 3.14159265358979323846 / 180.;
    static NeckLimits limits(const std::vector<Joint> &joints, const NeckEnvelopeConfig &envelope) {
        auto find = [&](const std::string &name) -> const JointConfig & {
            auto it = std::find_if(joints.begin(), joints.end(),
                                   [&](const auto &j) { return j.config.joint_name == name; });
            if (it == joints.end())
                throw std::invalid_argument("首制限にはpan/tilt両軸が必要です");
            return it->config;
        };
        const auto &pan = find("neck_pan_joint"), &tilt = find("neck_tilt_joint");
        if (!std::isfinite(envelope.pan_abs_at_outer) || envelope.pan_abs_at_outer <= 0 ||
            !std::isfinite(envelope.tilt_min_at_outer) || !std::isfinite(envelope.outward_margin) ||
            envelope.outward_margin < 0 || envelope.tilt_min_at_outer < tilt.limit_pos_min ||
            envelope.tilt_min_at_outer + envelope.outward_margin > tilt.limit_pos_max)
            throw std::invalid_argument("neck_limitsの設定が不足または不正です");
        return {pan.limit_pos_min, pan.limit_pos_max, tilt.limit_pos_min, tilt.limit_pos_max, envelope};
    }
    static double lower(double pan, const NeckLimits &limits) {
        return limits.tilt_min + (limits.envelope.tilt_min_at_outer - limits.tilt_min) *
                                     std::min(std::abs(pan) / limits.envelope.pan_abs_at_outer, 1.);
    }
    static std::array<double, 2> prepare(const std::array<double, 2> &measured,
                                         const std::array<double, 2> &previous, std::array<double, 2> target,
                                         const NeckLimits &limits) {
        for (const auto &q : {measured, previous, target})
            for (double v : q)
                if (!std::isfinite(v))
                    throw std::invalid_argument("首角度が非有限値です");
        target[0] = std::clamp(target[0], limits.pan_min, limits.pan_max);
        // 中央へ戻る途中も、実測と前回目標のうち最も外側のパンに合わせる。
        const double extent = std::max({std::abs(measured[0]), std::abs(previous[0]), std::abs(target[0])});
        const double required = lower(extent, limits);
        const bool outward = lower(target[0], limits) > lower(measured[0], limits) + 1e-10;
        // パンは待機させず、必要なチルト補正と旋回時の余裕を同じ周期で送る。
        target[1] = std::clamp(target[1], required + (outward ? limits.envelope.outward_margin : 0.),
                               limits.tilt_max);
        return target;
    }
};
} // namespace fuzzrobo::topoarm_hardware
