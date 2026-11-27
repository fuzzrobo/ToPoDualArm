// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/target_limiter.hpp"
#include <stdexcept>
namespace fuzzrobo::topoarm_hardware {
TargetLimiter::TargetLimiter(const std::vector<Joint> &joints, const SessionOptions &options,
                             Quantize quantize)
    : options_(options), quantize_(quantize) {
    if (options.neck_envelope) {
        neck_limits_ = NeckGuard::limits(joints, options.neck_limits);
        size_t j = 0;
        for (const auto &name : {"neck_pan_joint", "neck_tilt_joint"}) {
            auto it = std::find_if(joints.begin(), joints.end(),
                                   [&](const auto &d) { return d.config.joint_name == name; });
            if (it == joints.end())
                throw std::invalid_argument("首制限にはpan/tilt両軸が必要です");
            neck_indices_[j++] = size_t(it - joints.begin());
        }
    }
    if (options.floor_enabled) {
        floor_ = std::make_unique<FloorGuard>(options.robot_model, options.floor_z_m, options.floor_margin_m,
                                              options.robot_model_fragment);
        if (joints.size() != floor_->names().size())
            throw std::invalid_argument("床補正のURDFと機体設定の駆動軸数が一致しません");
        for (const auto &name : floor_->names()) {
            auto it = std::find_if(joints.begin(), joints.end(),
                                   [&](const auto &d) { return d.config.joint_name == name; });
            if (it == joints.end())
                throw std::invalid_argument("URDFと関節名が一致しません: " + name);
            floor_indices_.push_back(size_t(it - joints.begin()));
        }
    }
}
void TargetLimiter::reset(const std::vector<Joint> &joints) {
    if (floor_) {
        floor_previous_goal_.clear();
        for (size_t i : floor_indices_)
            floor_previous_goal_.push_back(joints[i].state.position);
    }
    if (options_.neck_envelope)
        for (size_t j = 0; j < 2; ++j)
            neck_previous_goal_[j] = joints[neck_indices_[j]].state.position;
}
void TargetLimiter::clear() { floor_previous_goal_.clear(); }
void TargetLimiter::commit(const std::vector<Joint> &joints) {
    if (options_.neck_envelope) {
        for (size_t j = 0; j < 2; ++j)
            neck_previous_goal_[j] = joints[neck_indices_[j]].command.position;
    }
    if (floor_) {
        floor_previous_goal_.clear();
        for (size_t i : floor_indices_)
            floor_previous_goal_.push_back(joints[i].command.position);
    }
}
void TargetLimiter::prepare_neck_targets(std::vector<Joint> &joints) {
    if (!options_.neck_envelope)
        return;
    std::array<double, 2> measured{}, target{};
    for (size_t j = 0; j < 2; ++j) {
        const auto &d = joints[neck_indices_[j]];
        measured[j] = d.state.position;
        target[j] = std::clamp(d.command.position, d.config.limit_pos_min, d.config.limit_pos_max);
    }
    target = NeckGuard::prepare(measured, neck_previous_goal_, target, neck_limits_);
    for (size_t j = 0; j < 2; ++j) {
        auto &d = joints[neck_indices_[j]];
        // 軸の静的範囲に合わない設定は、補正値を再クリップして制限を破らず停止する。
        if (target[j] < d.config.limit_pos_min - 1e-10 || target[j] > d.config.limit_pos_max + 1e-10)
            throw std::runtime_error("首の静的可動域と連動制限が両立しません");
        d.command.position = target[j];
    }
}
void TargetLimiter::prepare(std::vector<Joint> &joints) {
    prepare_neck_targets(joints);
    if (!floor_)
        return;
    std::vector<double> current, target;
    for (size_t i : floor_indices_) {
        const auto &d = joints[i];
        current.push_back(d.state.position);
        target.push_back(options_.clip_to_limits
                             ? std::clamp(d.command.position, d.config.limit_pos_min, d.config.limit_pos_max)
                             : d.command.position);
    }
    if (floor_previous_goal_.size() != current.size())
        throw std::runtime_error("床補正の保持目標が未初期化です");
    floor_result_ = floor_->track(current, floor_previous_goal_, target);
    // 実測が可動域端を僅かに越えていても、床補正後の目標は必ず関節範囲内に戻す。
    bool bounded_after_floor = false;
    for (size_t j = 0; j < floor_indices_.size(); ++j) {
        const auto &d = joints[floor_indices_[j]];
        double bounded =
            std::clamp(floor_result_.positions[j], d.config.limit_pos_min, d.config.limit_pos_max);
        bounded_after_floor |= std::abs(bounded - floor_result_.positions[j]) > 1e-12;
        floor_result_.positions[j] = bounded;
    }
    std::vector<double> quantized;
    for (size_t j = 0; j < floor_indices_.size(); ++j) {
        const auto &d = joints[floor_indices_[j]];
        const auto q = floor_result_.positions[j];
        quantized.push_back(quantize_ ? quantize_(d, q) : q);
    }
    const auto actual = floor_->clearance(current), command = floor_->clearance(quantized);
    for (size_t side = 0; side < 2; ++side)
        if (!floor_result_.holding[side] && command[side] + 1e-7 < std::min(actual[side], floor_->margin()))
            throw std::runtime_error("量子化後の目標が床余裕を下回ります");
    if (bounded_after_floor) {
        // 保持中でも関節端で目標を変えた場合は、保持の例外を適用しない。
        // 関節端への戻しが床補正を打ち消さないよう、その最終経路を検査する。
        double span = 0;
        for (size_t j = 0; j < current.size(); ++j)
            span = std::max(span, std::abs(quantized[j] - current[j]));
        const int count = std::max(1, int(std::ceil(span / (.25 * 3.14159265358979323846 / 180))));
        for (int k = 1; k <= count; ++k) {
            auto q = current;
            for (size_t j = 0; j < q.size(); ++j)
                q[j] += double(k) / count * (quantized[j] - current[j]);
            auto d = floor_->clearance(q);
            for (size_t side = 0; side < 2; ++side)
                if (d[side] + 1e-7 < std::min(actual[side], floor_->margin()))
                    throw std::runtime_error("関節制限と床余裕を同時に満たす目標を生成できません");
        }
    }
    if (options_.neck_envelope) {
        std::array<double, 2> measured{}, candidate{};
        for (size_t j = 0; j < 2; ++j) {
            measured[j] = joints[neck_indices_[j]].state.position;
            auto it = std::find(floor_indices_.begin(), floor_indices_.end(), neck_indices_[j]);
            candidate[j] = floor_result_.positions[size_t(it - floor_indices_.begin())];
        }
        const auto checked = NeckGuard::prepare(measured, neck_previous_goal_, candidate, neck_limits_);
        for (size_t j = 0; j < 2; ++j)
            if (std::abs(checked[j] - candidate[j]) > 1e-8)
                throw std::runtime_error("床補正と首の連動制限を同時に満たせません");
    }
    floor_result_.after = floor_->clearance(floor_result_.positions);
    for (size_t j = 0; j < floor_indices_.size(); ++j)
        joints[floor_indices_[j]].command.position = floor_result_.positions[j];
}
} // namespace fuzzrobo::topoarm_hardware
