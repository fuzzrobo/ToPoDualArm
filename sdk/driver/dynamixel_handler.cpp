// Copyright 2025 FuzzRoBo CO., LTD.
// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/dynamixel_handler.hpp"
#include <initializer_list>
#include <fcntl.h>
#include <set>
#include <stdexcept>
#include <sys/file.h>
#include <unistd.h>
#include <chrono>
#include <thread>

namespace fuzzrobo::topoarm_hardware {
namespace {
// 上限値を生値へ戻す際のceil誤差と、1カウント未満の切上げによる上限超過を避ける。
int64_t bounded_current_raw(const Joint &d) {
    auto raw = AddrX::goal_current.val2pulse(d.command.current_limit * 1000, d.state.model_number);
    const double limit = std::min(d.command.current_limit, d.config.current_cap);
    while (raw > 0 && AddrX::goal_current.pulse2val(raw, d.state.model_number) * .001 > limit)
        --raw;
    return raw;
}
class LibTransport final : public Transport {
    // communicatorが保持するconst char*の寿命を保証する。
    std::string port_;
    DynamixelCommunicator bus_;
    int lock_fd_{-1};
    static std::string checked_port(const std::string &port, int baud, int latency) {
        if (port.empty() || port.size() >= 100 || baud <= 0 || latency < 1)
            throw std::invalid_argument("ポート・baud・latency設定が不正です");
        return port;
    }

  public:
    LibTransport(const std::string &port, int baud, int latency)
        : port_(checked_port(port, baud, latency)), bus_(port_.c_str(), baud, latency) {}
    ~LibTransport() override { close(); }
    bool open() override {
        if (lock_fd_ >= 0)
            return false;
        lock_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (lock_fd_ < 0)
            return false;
        if (flock(lock_fd_, LOCK_EX | LOCK_NB) != 0 || !bus_.OpenPort()) {
            close();
            return false;
        }
        return true;
    }
    void close() override {
        bus_.ClosePort();
        if (lock_fd_ >= 0) {
            ::close(lock_fd_);
            lock_fd_ = -1;
        }
    }
    bool ping(uint8_t id) override { return bus_.Ping(id); }
    bool reboot(uint8_t id) override {
        bus_.Reboot(id);
        // 再起動中に通常の読書きを送らない。再試行せず、1回のPingで確認する。
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return bus_.Ping(id) && healthy();
    }
    bool healthy() override { return !bus_.timeout_last_read() && !bus_.comm_error_last_read(); }
    bool hardware_error() override { return bus_.hardware_error_last_read(); }
    int64_t read(DynamixelAddress a, uint8_t id) override { return bus_.Read(a, id); }
    bool write(DynamixelAddress a, uint8_t id, int64_t v) override { return bus_.Write(a, id, v); }
    std::map<uint8_t, std::vector<int64_t>> sync_read(const std::vector<DynamixelAddress> &a,
                                                      const std::vector<uint8_t> &ids) override {
        return bus_.SyncRead(a, ids);
    }
    bool sync_write(DynamixelAddress a, const std::vector<uint8_t> &ids,
                    const std::vector<int64_t> &v) override {
        return bus_.SyncWrite(a, ids, v);
    }
};
} // namespace
DynamixelHandler::DynamixelHandler(const std::string &port, int baud, int latency)
    : DynamixelHandler(std::make_unique<LibTransport>(port, baud, latency)) {}
DynamixelHandler::DynamixelHandler(std::unique_ptr<Transport> t) : transport_(std::move(t)) {}
DynamixelHandler::~DynamixelHandler() {
    if (transport_)
        transport_->close();
}
bool DynamixelHandler::fail(BusError code, int id, const std::string &detail) {
    last_result_ = {code, id, detail};
    return false;
}
bool DynamixelHandler::write_verified(DynamixelAddress address, uint8_t id, int64_t value,
                                      const std::string &context, bool check_alert) {
    if (!transport_->write(address, id, value))
        return fail(BusError::Communication, id, context + "書込み失敗");
    const auto actual = transport_->read(address, id);
    if (!transport_->healthy() || (check_alert && transport_->hardware_error()) || actual != value)
        return fail(BusError::Communication, id, context + "読戻し不一致");
    return true;
}
bool DynamixelHandler::open_port() { return transport_->open(); }
bool DynamixelHandler::close_port() {
    transport_->close();
    return true;
}
BusResult DynamixelHandler::scan(const std::vector<uint8_t> &ids, const std::vector<std::string> &names) {
    last_result_ = {};
    if (ids.empty() || ids.size() != names.size() ||
        std::set<uint8_t>(ids.begin(), ids.end()).size() != ids.size()) {
        fail(BusError::Configuration, -1, "ID・関節名の対応が不正です");
        return last_result_;
    }
    std::vector<Joint> found;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == 0 || ids[i] > 253 || !transport_->ping(ids[i])) {
            fail(BusError::Communication, ids[i], "Ping失敗");
            return last_result_;
        }
        const auto model = transport_->read(AddrCommon::model_number, ids[i]);
        if (!transport_->healthy() || model <= 0 || model > 65535) {
            fail(BusError::Communication, ids[i], "機種読出し失敗");
            return last_result_;
        }
        Joint d;
        d.config.id = ids[i];
        d.config.joint_name = names[i];
        d.state.model_number = model;
        if (dynamixel_series(d.state.model_number) != SERIES_X) {
            fail(BusError::Configuration, ids[i], "Xシリーズ以外は未対応です");
            return last_result_;
        }
        if (!has_current_sensor(d.state.model_number)) {
            fail(BusError::Configuration, ids[i],
                 "電流センサなしの機種は未対応です。LoadをAとして扱いません");
            return last_result_;
        }
        found.push_back(d);
    }
    // export_state_interfaces後のポインタを無効にしない。
    if (dxls.empty()) {
        dxls = std::move(found);
    } else {
        if (dxls.size() != found.size()) {
            fail(BusError::Configuration, -1, "再接続時の軸数が不一致です");
            return last_result_;
        }
        for (size_t i = 0; i < found.size(); ++i) {
            if (dxls[i].config.id != found[i].config.id ||
                dxls[i].config.joint_name != found[i].config.joint_name) {
                fail(BusError::Configuration, found[i].config.id, "再接続時の対応が不一致です");
                return last_result_;
            }
        }
        for (size_t i = 0; i < found.size(); ++i)
            dxls[i].state.model_number = found[i].state.model_number;
    }
    dxl_ids = ids;
    return last_result_;
}
BusResult DynamixelHandler::set_torque(bool enabled) {
    last_result_ = {};
    for (auto id : dxl_ids) {
        if (!write_verified(AddrX::torque_enable, id, enabled ? 1 : 0, "Torque", false))
            return last_result_;
    }
    return last_result_;
}
bool DynamixelHandler::write_position_pid() {
    last_result_ = {};
    // RAM設定のみ。Torque・動作モード・電流制限には触れない。
    for (const auto &d : dxls) {
        for (const auto &[address, value] : std::vector<std::pair<DynamixelAddress, int64_t>>{
                 {AddrX::position_p_gain, d.config.position_kp},
                 {AddrX::position_i_gain, d.config.position_ki},
                 {AddrX::position_d_gain, d.config.position_kd}}) {
            if (!write_verified(address, d.config.id, value, "位置PID"))
                return false;
        }
    }
    return true;
}
BusResult DynamixelHandler::configure(bool allow_changes) {
    last_result_ = {};
    for (auto &d : dxls)
        if (!configure_joint(d, allow_changes))
            break;
    return last_result_;
}
bool DynamixelHandler::configure_joint(Joint &d, bool allow_changes) {
    // 設定照合だけならTorque保持中でも再接続できる。設定変更時だけOFFを要求する。
    const auto torque = transport_->read(AddrX::torque_enable, d.config.id);
    if (!transport_->healthy())
        return fail(BusError::Communication, d.config.id, "Torque状態読出し失敗");
    if (torque != 0 && allow_changes)
        return fail(BusError::Configuration, d.config.id, "設定変更にはTorque OFFが必要です");
    for (const auto &[address, desired] : std::initializer_list<std::pair<DynamixelAddress, int64_t>>{
             {AddrX::operating_mode, d.config.operating_mode},
             {AddrX::position_p_gain, d.config.position_kp},
             {AddrX::position_i_gain, d.config.position_ki},
             {AddrX::position_d_gain, d.config.position_kd}}) {
        const auto actual = transport_->read(address, d.config.id);
        if (!transport_->healthy())
            return fail(BusError::Communication, d.config.id, "設定読出し失敗");
        if (actual == desired)
            continue;
        if (!allow_changes || !transport_->write(address, d.config.id, desired))
            return fail(BusError::Configuration, d.config.id, "設定不一致。自動変更は既定で禁止です");
        const auto confirmed = transport_->read(address, d.config.id);
        if (!transport_->healthy() || confirmed != desired)
            return fail(BusError::Communication, d.config.id, "設定読戻し不一致");
    }
    const auto raw_limit = transport_->read(AddrX::current_limit, d.config.id);
    if (!transport_->healthy() || raw_limit <= 0)
        return fail(BusError::Configuration, d.config.id, "電流上限読出し失敗");
    const double cap = AddrX::goal_current.pulse2val(raw_limit, d.state.model_number) * 0.001;
    if (!std::isfinite(cap) || cap <= 0 || !std::isfinite(d.config.current_cap) || d.config.current_cap <= 0)
        return fail(BusError::Configuration, d.config.id, "電流上限が不正です");
    d.config.current_cap = std::min(d.config.current_cap, cap);
    d.command.current_limit = std::min(d.command.current_limit, d.config.current_cap);
    auto drive = transport_->read(AddrX::drive_mode, d.config.id);
    if (!transport_->healthy() || transport_->hardware_error())
        return fail(BusError::Communication, d.config.id, "Drive Mode読取り失敗");
    const bool timed = d.config.profile_duration_ms >= 0;
    if (timed && !(drive & 4)) {
        // 明示的な時間設定に対応するbitだけ変更する。自動Torque OFF/ONは行わない。
        if (torque != 0)
            return fail(BusError::Configuration, d.config.id, "ID=" + std::to_string(d.config.id) +
                " 時間モードへの初回切替にはTorque OFFが必要です（電源再投入後に起動）");
        const auto desired = drive | 4;
        if (!write_verified(AddrX::drive_mode, d.config.id, desired, "時間モード"))
            return false;
        drive = desired;
    }
    if (!timed && (drive & 4) && (d.config.profile_vel != 0 || d.config.profile_acc != 0))
        return fail(BusError::Configuration, d.config.id, "プロファイル単位とDrive Modeが不一致です");
    // 時間ベースはms、速度ベースは既存の物理単位。読戻しで方式を混同しない。
    for (const auto &[address, raw] : std::initializer_list<std::pair<DynamixelAddress, int64_t>>{
             {AddrX::profile_velocity, timed ? d.config.profile_duration_ms :
                 AddrX::profile_velocity.val2pulse(d.config.profile_vel, d.state.model_number)},
             {AddrX::profile_acceleration, timed ? d.config.profile_acceleration_ms :
                 AddrX::profile_acceleration.val2pulse(d.config.profile_acc, d.state.model_number)}}) {
        const auto previous = transport_->read(address, d.config.id);
        if (!transport_->healthy() || raw < 0)
            return fail(BusError::Communication, d.config.id, "プロファイル読出し失敗");
        if (previous != raw &&
            (!(allow_changes || timed) || !transport_->write(address, d.config.id, raw)))
            return fail(BusError::Configuration, d.config.id, "プロファイル設定失敗");
        const auto actual = transport_->read(address, d.config.id);
        if (!transport_->healthy() || transport_->hardware_error() || actual != raw)
            return fail(BusError::Communication, d.config.id, "プロファイル読戻し不一致");
    }
    return true;
}
bool DynamixelHandler::configure_return_delay_time(int raw) {
    last_result_ = {};
    if (raw < 0 || raw > 254)
        return fail(BusError::Configuration, -1, "Return Delay Timeが不正です");
    std::vector<uint8_t> changes;
    for (auto id : dxl_ids) {
        const auto previous = transport_->read(AddrX::return_delay_time, id);
        if (!transport_->healthy() || transport_->hardware_error())
            return fail(BusError::Communication, id, "Return Delay Time読取り失敗");
        if (previous == raw)
            continue; // EEPROMを起動のたびに書き換えない。
        const auto torque = transport_->read(AddrX::torque_enable, id);
        if (!transport_->healthy() || transport_->hardware_error() || torque != 0)
            return fail(BusError::Configuration, id, "Return Delay Time変更にはTorque OFFが必要です");
        changes.push_back(id);
    }
    for (auto id : changes) {
        if (!write_verified(AddrX::return_delay_time, id, raw, "Return Delay Time"))
            return false;
    }
    return true;
}
bool DynamixelHandler::read_all() {
    last_result_ = {};
    faults_.clear();
    // 内部軌道も同じ64〜146番地の範囲に含める。通信回数・読出しバイト数は増やさない。
    const std::vector<DynamixelAddress> addresses{AddrX::present_position,      AddrX::present_velocity,
                                                  AddrX::present_current,       AddrX::present_pwm,
                                                  AddrX::present_input_voltage, AddrX::present_temperature,
                                                  AddrX::torque_enable,         AddrX::hardware_error_status,
                                                  AddrX::realtime_tick,         AddrX::goal_position,
                                                  AddrX::position_trajectory,   AddrX::velocity_trajectory};
    const auto values = transport_->sync_read(addresses, dxl_ids);
    if (!transport_->healthy() || values.size() != dxls.size())
        return fail(BusError::Incomplete, -1, "読出し周期が欠落しました");
    for (const auto &d : dxls) {
        const auto it = values.find(d.config.id);
        if (it == values.end() || it->second.size() != addresses.size())
            return fail(BusError::Incomplete, d.config.id, "IDまたは項目が欠落しました");
    }
    // 欠測がないことを先に確認し、異常IDをすべて保存する。異常値は公開状態へ混ぜない。
    for (const auto &d : dxls)
        if (values.at(d.config.id)[7] != 0)
            faults_.push_back({d.config.id, values.at(d.config.id)[7]});
    if (!faults_.empty())
        return fail(BusError::Hardware, faults_[0].id,
                    "サーボのhardware error: ID=" + std::to_string(faults_[0].id) +
                    " status=" + std::to_string(faults_[0].error));
    if (transport_->hardware_error())
        return fail(BusError::Hardware, -1, "サーボのAlertビット");
    // 全軸を検証してから公開状態を更新する。
    for (auto &d : dxls) {
        const auto &v = values.at(d.config.id);
        d.state.position_raw = v[0];
        d.state.position =
            addresses[0].pulse2val(v[0], d.state.model_number) * d.config.multiplier + d.config.offset;
        d.state.velocity = addresses[1].pulse2val(v[1], d.state.model_number) * d.config.multiplier;
        d.state.current = addresses[2].pulse2val(v[2], d.state.model_number) * 0.001;
        d.state.pwm = addresses[3].pulse2val(v[3], d.state.model_number);
        d.state.voltage = static_cast<double>(v[4]) * 0.1;
        d.state.temperature = v[5];
        d.state.torque_enabled = v[6];
        d.state.hardware_error = v[7];
        d.state.realtime_tick = v[8];
        d.state.goal_position_readback =
            addresses[9].pulse2val(v[9], d.state.model_number) * d.config.multiplier + d.config.offset;
        d.state.position_trajectory =
            addresses[10].pulse2val(v[10], d.state.model_number) * d.config.multiplier + d.config.offset;
        d.state.velocity_trajectory =
            addresses[11].pulse2val(v[11], d.state.model_number) * d.config.multiplier;
        d.state.valid = 1.0;
        d.state.age = 0.0;
    }
    return true;
}
bool DynamixelHandler::reset_conditions(uint8_t id, bool expect_clear) {
    auto read = [&](DynamixelAddress a, int64_t &out) {
        out = transport_->read(a, id);
        return transport_->healthy();
    };
    int64_t error, voltage, temperature, min_voltage, max_voltage, max_temperature, drive, startup;
    if (!read(AddrX::hardware_error_status, error) ||
        !read(AddrX::present_input_voltage, voltage) ||
        !read(AddrX::present_temperature, temperature) ||
        !read(AddrX::min_voltage_limit, min_voltage) ||
        !read(AddrX::max_voltage_limit, max_voltage) ||
        !read(AddrX::temperature_limit, max_temperature) || !read(AddrX::drive_mode, drive) ||
        !read(AddrX::startup_configuration, startup))
        return fail(BusError::Communication, id, "自動復旧の状態確認失敗");
    if ((expect_clear ? error != 0 : error <= 0 || (error & ~0x21) != 0) ||
        min_voltage <= 0 || max_voltage <= min_voltage ||
        voltage <= min_voltage || voltage >= max_voltage ||
        max_temperature <= 0 || temperature < 0 || temperature >= max_temperature ||
        (drive & 8) || startup != 0)
        return fail(BusError::Hardware, id, "自動復旧対象外の異常・電圧・温度・自動Torque設定");
    return true;
}
bool DynamixelHandler::recover_faults() {
    if (faults_.empty())
        return fail(BusError::Hardware, -1, "自動復旧対象のIDがありません");
    const auto faults = faults_;
    // 他の異常や通信断が混在したら、どのIDにもTorque OFF/REBOOTを送らない。
    for (const auto &fault : faults) {
        const auto it = std::find_if(dxls.begin(), dxls.end(), [&](const Joint &j) { return j.config.id == fault.id; });
        if (it == dxls.end() || !std::isfinite(it->command.current_limit) ||
            it->command.current_limit < 0 || it->command.current_limit > it->config.current_cap)
            return fail(BusError::InvalidCommand, fault.id, "復旧時の電流上限が不正です");
        if (fault.error <= 0 || (fault.error & ~0x21) != 0)
            return fail(BusError::Hardware, fault.id, "自動復旧対象外のhardware error");
        if (!reset_conditions(fault.id, false))
            return false;
    }
    // 設定復元に失敗した対象IDへONは送らない。正常IDのTorqueや設定には触れない。
    for (const auto &fault : faults) {
        auto &d = *std::find_if(dxls.begin(), dxls.end(), [&](const Joint &j) { return j.config.id == fault.id; });
        const auto id = fault.id;
        if (!write_verified(AddrX::torque_enable, id, 0, "復旧前Torque OFF", false))
            return false;
        if (!transport_->reboot(id))
            return fail(BusError::Communication, id, "REBOOT後のPing失敗");
        if (!reset_conditions(id, true))
            return false;
        const auto torque = transport_->read(AddrX::torque_enable, id);
        if (!transport_->healthy() || torque != 0)
            return fail(BusError::Hardware, id, "REBOOT後にTorque OFFを確認できません");
        if (!configure_joint(d, true))
            return false;
        if (d.config.operating_mode == 5 &&
            !write_verified(AddrX::goal_current, id, bounded_current_raw(d), "復旧電流上限"))
            return false;
    }
    // 他IDのREBOOT待機中にも脱力姿勢は変わるため、各ONの直前に位置を読み直す。
    for (const auto &fault : faults) {
        auto &d = *std::find_if(dxls.begin(), dxls.end(), [&](const Joint &j) { return j.config.id == fault.id; });
        const auto id = fault.id;
        if (!reset_conditions(id, true))
            return false;
        const auto raw = transport_->read(AddrX::present_position, id);
        if (!transport_->healthy() || transport_->hardware_error() || raw < 0 || raw > 4095)
            return fail(BusError::Hardware, id, "復旧時の現在位置を確認できません");
        if (!write_verified(AddrX::goal_position, id, raw, "復旧時の現在位置保持"))
            return false;
        // 旧目標・旧量子化キャッシュを再利用しない。
        d.command.position = AddrX::present_position.pulse2val(raw, d.state.model_number) *
                                 d.config.multiplier + d.config.offset;
        d.sent = {};
        if (!write_verified(AddrX::torque_enable, id, 1, "復旧Torque ON"))
            return false;
    }
    last_result_ = {};
    return true;
}
bool DynamixelHandler::set_joints_current() {
    std::vector<uint8_t> ids;
    std::vector<int64_t> raw;
    for (const auto &d : dxls) {
        if (d.config.operating_mode != 5)
            continue;
        if (!std::isfinite(d.command.current_limit) || d.command.current_limit < 0 ||
            d.command.current_limit > d.config.current_cap)
            return fail(BusError::InvalidCommand, d.config.id, "電流上限が承認範囲外です");
        ids.push_back(d.config.id);
        raw.push_back(bounded_current_raw(d));
    }
    if (!ids.empty() && !transport_->sync_write(AddrX::goal_current, ids, raw))
        return fail(BusError::Communication, -1, "電流上限SyncWrite失敗");
    last_result_ = {};
    return true;
}
bool DynamixelHandler::check_torque_off_target_write() {
    // 目標更新による自動Torque ONが有効な機器には、診断指令も送らない。
    for (auto id : dxl_ids) {
        const auto torque = transport_->read(AddrX::torque_enable, id);
        if (!transport_->healthy() || transport_->hardware_error() || torque != 0)
            return fail(BusError::Configuration, id, "Torque OFF診断の状態確認失敗");
        const auto drive = transport_->read(AddrX::drive_mode, id);
        if (!transport_->healthy() || transport_->hardware_error() || (drive & 8))
            return fail(BusError::Configuration, id, "目標更新による自動Torque ON設定、または読取り失敗");
    }
    last_result_ = {};
    return true;
}
int64_t DynamixelHandler::position_goal_pulse(const Joint &d, double position) {
    // 同じ保持目標は、実測が動いても最後に送った生値を再利用する。
    if (d.sent.valid && position == d.sent.position)
        return d.sent.raw;
    // 実測位置そのものを保持する場合は、浮動小数の往復で1pulse進めない。
    if (d.state.valid == 1 && std::abs(position - d.state.position) < 1e-12)
        return d.state.position_raw;
    return AddrX::goal_position.val2pulse((position - d.config.offset) / d.config.multiplier,
                                          d.state.model_number);
}
bool DynamixelHandler::set_joints_position(bool acknowledged) {
    std::vector<int64_t> raw;
    for (const auto &d : dxls) {
        if (!std::isfinite(d.command.position) || d.command.position < d.config.limit_pos_min ||
            d.command.position > d.config.limit_pos_max || !std::isfinite(d.config.multiplier) ||
            d.config.multiplier == 0)
            return fail(BusError::InvalidCommand, d.config.id, "位置目標・変換が不正です");
        auto pulse = position_goal_pulse(d, d.command.position);
        // エンコーダ量子化後も静的可動域の内側へ収める（例: 上限105度）。
        auto angle = [&](int64_t value) {
            return AddrX::goal_position.pulse2val(value, d.state.model_number) * d.config.multiplier +
                   d.config.offset;
        };
        const int step = d.config.multiplier > 0 ? 1 : -1;
        if (angle(pulse) < d.config.limit_pos_min)
            pulse += step;
        if (angle(pulse) > d.config.limit_pos_max)
            pulse -= step;
        if (angle(pulse) < d.config.limit_pos_min - 1e-10 || angle(pulse) > d.config.limit_pos_max + 1e-10)
            return fail(BusError::InvalidCommand, d.config.id, "量子化後の位置が可動域内に収まりません");
        raw.push_back(pulse);
    }
    if (acknowledged) {
        // Torque OFF診断では各軸のWrite応答も確認する。通常追従はSyncWriteを維持。
        for (size_t i = 0; i < dxl_ids.size(); ++i)
            if (!transport_->write(AddrX::goal_position, dxl_ids[i], raw[i]))
                return fail(BusError::Communication, dxl_ids[i], "位置Writeの応答確認失敗");
    } else if (!transport_->sync_write(AddrX::goal_position, dxl_ids, raw))
        return fail(BusError::Communication, -1, "位置SyncWrite失敗");
    for (size_t i = 0; i < dxls.size(); ++i) {
        dxls[i].sent.valid = true;
        dxls[i].sent.position = dxls[i].command.position;
        dxls[i].sent.raw = raw[i];
    }
    last_result_ = {};
    return true;
}
bool DynamixelHandler::set_goal_to_present() {
    // 起動時は現在位置の生値をそのまま目標へ書く。radの往復変換をしない。
    std::vector<int64_t> raw;
    for (const auto &d : dxls)
        raw.push_back(d.state.position_raw);
    if (!transport_->sync_write(AddrX::goal_position, dxl_ids, raw))
        return fail(BusError::Communication, -1, "現在位置の目標書込み失敗");
    for (auto &d : dxls) {
        d.sent.valid = true;
        d.sent.position = d.state.position;
        d.sent.raw = d.state.position_raw;
    }
    last_result_ = {};
    return true;
}
bool DynamixelHandler::verify_hold() {
    for (const auto &d : dxls) {
        const auto actual = transport_->read(AddrX::goal_position, d.config.id);
        const auto wanted = position_goal_pulse(d, d.command.position);
        if (!transport_->healthy() || actual != wanted)
            return fail(BusError::Communication, d.config.id,
                        "保持位置の読戻し不一致: expected=" + std::to_string(wanted) +
                            " actual=" + std::to_string(actual));
        if (d.config.operating_mode == 5) {
            const auto current = transport_->read(AddrX::goal_current, d.config.id);
            const auto expected = bounded_current_raw(d);
            if (!transport_->healthy() || current != expected)
                return fail(BusError::Communication, d.config.id,
                            "保持電流上限の読戻し不一致: expected=" + std::to_string(expected) +
                                " actual=" + std::to_string(current));
        }
    }
    last_result_ = {};
    return true;
}
} // namespace fuzzrobo::topoarm_hardware
