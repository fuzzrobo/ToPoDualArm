// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/driver_session.hpp"
#include "topoarm_hardware/settings.hpp"
#include <limits>
#include <stdexcept>

namespace fuzzrobo::topoarm_hardware {
namespace {
double seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace
void DriverSession::validate(const std::vector<Joint> &joints, const SessionOptions &options) {
    validate_settings(joints, options);
}
DriverSession::DriverSession(DynamixelHandler &handler, SessionOptions options)
    : handler_(handler), options_(options) {
    validate(handler.dxls, options);
    limiter_ = std::make_unique<TargetLimiter>(handler.dxls, options, [](const Joint &d, double q) {
        const auto pulse = DynamixelHandler::position_goal_pulse(d, q);
        return AddrX::goal_position.pulse2val(pulse, d.state.model_number) * d.config.multiplier + d.config.offset;
    });
}
bool DriverSession::fail(const std::string &reason) {
    error_ = reason;
    fresh_ = false;
    write_ok = 0;
    for (auto &d : handler_.dxls) {
        d.state.valid = 0;
        d.state.age = std::numeric_limits<double>::quiet_NaN();
    }
    return false;
}
bool DriverSession::positions_valid() const {
    for (const auto &d : handler_.dxls)
        if (!std::isfinite(d.state.position) ||
            (!options_.clip_to_limits &&
             (d.state.position < d.config.limit_pos_min || d.state.position > d.config.limit_pos_max)))
            return false;
    return true;
}
bool DriverSession::configure(bool passive) {
    if (connected_) {
        error_ = "接続済みです";
        return false;
    }
    if (!options_.verified) {
        error_ = "hardware_verifiedの確認が必要です";
        return false;
    }
    if (!handler_.open_port())
        return fail("ポートを開けません（使用中または権限不足）");
    reset_attempted_.clear();
    last_reset_ids_.clear();
    reset_count_ = 0;
    reset_pending_ = false;
    std::vector<std::string> names;
    for (const auto &d : handler_.dxls)
        names.push_back(d.config.joint_name);
    if (!handler_.scan(handler_.dxl_ids, names)) {
        handler_.close_port();
        return fail("機種確認失敗: " + handler_.last_result().detail);
    }
    apply_model_overrides(handler_.dxls, options_);
    // YAMLと部分指定を合成した結果も、レジスタ書込み前に検証する。
    try {
        validate(handler_.dxls, options_);
    } catch (const std::invalid_argument &e) {
        handler_.close_port();
        return fail(e.what());
    }
    const bool pid_override =
        options_.xm540_p >= 0 || options_.xm540_d >= 0 || options_.xm430_p >= 0 || options_.xm430_d >= 0;
    // 明示指定されたReturn Delay Timeは読み取り接続でも設定する。
    // PID・時間設定は明示指定に従い、それ以外の変更はconfigure_devices指定時だけ行う。
    if ((!passive && options_.return_delay_time >= 0 &&
         !handler_.configure_return_delay_time(options_.return_delay_time)) ||
        (!passive && options_.torque && (options_.write_pid_on_connect || pid_override) &&
         !handler_.write_position_pid()) ||
        (!passive && (options_.torque || options_.configure_devices) &&
         !handler_.configure(options_.configure_devices)) ||
        !handler_.read_all() || !positions_valid()) {
        handler_.close_port();
        return fail("初期状態・設定の確認失敗: " + handler_.last_result().detail);
    }
    connected_ = true;
    fresh_ = true;
    error_.clear();
    return true;
}
bool DriverSession::enable_passive() {
    if (!connected_ || active_ || !options_.verified || !options_.torque || options_.configure_devices)
        return fail("駆動開始には確認済みの受動接続が必要です");
    const bool pid_override = options_.xm540_p >= 0 || options_.xm540_d >= 0 ||
                              options_.xm430_p >= 0 || options_.xm430_d >= 0;
    if ((options_.return_delay_time >= 0 && !handler_.configure_return_delay_time(options_.return_delay_time)) ||
        ((options_.write_pid_on_connect || pid_override) && !handler_.write_position_pid()) ||
        !handler_.configure(false))
        return fail("サーボ設定の検証失敗: " + handler_.last_result().detail);
    return activate();
}
bool DriverSession::activate() {
    if (!connected_)
        return fail("接続されていません");
    if (active_)
        return true;
    if (!handler_.read_all() || !positions_valid())
        return fail("起動時の状態確認失敗");
    for (auto &d : handler_.dxls)
        d.command.position = options_.clip_to_limits ? std::clamp(d.state.position, d.config.limit_pos_min,
                                                                  d.config.limit_pos_max)
                                                     : d.state.position;
    // 現在位置を目標にしてTorque ONする。以後はleader由来の目標を送る。
    // 可動域制限は以後の指令に適用し、起動時に境界へ移動させない。
    if (options_.torque &&
        (!handler_.set_joints_current() || !handler_.set_goal_to_present() || !handler_.set_torque(true)))
        return fail("現在位置設定・Torque ON失敗: ID=" + std::to_string(handler_.last_result().id) + " " +
                    handler_.last_result().detail);
    limiter_->reset(handler_.dxls);
    active_ = true;
    fresh_ = true;
    error_.clear();
    return true;
}
bool DriverSession::deactivate() {
    const bool was_active = active_;
    active_ = false;
    // 既定は保持。自動脱力による重力落下を避ける。
    if (was_active && options_.torque && !options_.retention && !handler_.set_torque(false))
        return fail("Torque OFF確認失敗");
    return true;
}
void DriverSession::close() {
    active_ = false;
    fresh_ = false;
    connected_ = false;
    limiter_->clear();
    handler_.close_port();
}
void DriverSession::abort() {
    fail("外部エラーにより停止しました");
    close();
}
bool DriverSession::read() {
    if (!connected_)
        return fail("接続されていません");
    read_start = seconds();
    bool ok = handler_.read_all();
    if (!ok && options_.auto_reset && options_.torque && active_ &&
        handler_.last_result().error == BusError::Hardware && !handler_.faults().empty()) {
        std::vector<uint8_t> ids;
        for (const auto &fault : handler_.faults()) {
            if (reset_attempted_.count(fault.id))
                return fail("同じIDの異常が再発しました。自動復旧は接続中1回だけです: ID=" +
                            std::to_string(fault.id));
            ids.push_back(fault.id);
        }
        reset_attempted_.insert(ids.begin(), ids.end());
        fresh_ = false;
        write_ok = 0;
        if (!handler_.recover_faults())
            return fail("自動復旧失敗: ID=" + std::to_string(handler_.last_result().id) + " " +
                        handler_.last_result().detail);
        ok = handler_.read_all();
        if (ok) {
            for (auto &d : handler_.dxls) {
                d.command.position = d.state.position;
                d.sent = {};
            }
            limiter_->reset(handler_.dxls);
            last_reset_ids_ = std::move(ids);
            ++reset_count_;
            reset_pending_ = true;
        }
    }
    read_end = seconds();
    if (!ok) {
        return fail(handler_.last_result().detail);
    }
    if (!positions_valid())
        return fail("実測位置がソフト可動域外です");
    if (active_ && options_.torque)
        for (const auto &d : handler_.dxls)
            if (d.state.torque_enabled != 1)
                return fail("Torqueが予期せずOFFになりました");
    fresh_ = true;
    error_.clear();
    return true;
}
bool DriverSession::resume_after_reset() {
    if (!connected_ || !active_ || !fresh_)
        return fail("復旧後の再開には起動中の新しい実測値が必要です");
    if (reset_pending_) {
        for (auto &d : handler_.dxls) {
            d.command.position = d.state.position;
            d.sent = {};
        }
        limiter_->reset(handler_.dxls);
    }
    reset_pending_ = false;
    return true;
}
bool DriverSession::prepare_floor_targets() {
    if (options_.neck_envelope && (!active_ || !fresh_))
        return fail("首制限には起動後の新しい実測値が必要です");
    if (options_.floor_enabled && (!connected_ || !fresh_))
        return fail("床補正には新しい実測値が必要です");
    try {
        limiter_->prepare(handler_.dxls);
        return true;
    } catch (const std::exception &error) {
        return fail(error.what());
    }
}
bool DriverSession::write() { return write_impl(false); }
bool DriverSession::write_torque_off() { return write_impl(true); }
bool DriverSession::write_impl(bool torque_off_diagnostic) {
    write_ok = 0;
    if (!connected_ || !active_)
        return fail("起動されていません");
    if (reset_pending_)
        return fail("自動復旧後は現在位置で保持中です。追従再開を明示してください");
    if (torque_off_diagnostic) {
        if (options_.torque || options_.configure_devices)
            return fail("Torque OFF診断には書込み設定なし・torque=falseの接続が必要です");
        if (!read())
            return false;
        for (const auto &d : handler_.dxls)
            if (d.state.torque_enabled != 0)
                return fail("Torque OFF診断中にTorque ONを検出しました: " + d.config.joint_name);
    } else if (!options_.torque)
        return true;
    if (!fresh_)
        return fail("有効な状態を読み直してください");
    for (size_t i = 0; i < handler_.dxls.size(); ++i) {
        const auto &d = handler_.dxls[i];
        if (!std::isfinite(d.command.position) ||
            (!options_.clip_to_limits &&
             (d.command.position < d.config.limit_pos_min || d.command.position > d.config.limit_pos_max)) ||
            !std::isfinite(d.command.current_limit) || d.command.current_limit < 0 ||
            d.command.current_limit > d.config.current_cap)
            return fail("位置・電流上限の指令検証に失敗しました");
    }
    if (options_.clip_to_limits)
        for (auto &d : handler_.dxls)
            d.command.position =
                std::clamp(d.command.position, d.config.limit_pos_min, d.config.limit_pos_max);
    if (!prepare_floor_targets())
        return false;
    if (torque_off_diagnostic && !handler_.check_torque_off_target_write())
        return fail(handler_.last_result().detail);
    write_start = seconds();
    const bool ok = handler_.set_joints_current() && handler_.set_joints_position(torque_off_diagnostic);
    write_end = seconds();
    if (!ok)
        return fail(handler_.last_result().detail);
    write_ok = 1;
    error_.clear();
    limiter_->commit(handler_.dxls);
    if (options_.neck_envelope || options_.floor_enabled)
        fresh_ = false;
    return true;
}
} // namespace fuzzrobo::topoarm_hardware
