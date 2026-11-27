// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/vsido.hpp"
#include "topoarm_hardware/settings.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace fuzzrobo::topoarm_hardware {
namespace {
constexpr double rads_per_count = 3.14159265358979323846 / 1800;
constexpr double nan = std::numeric_limits<double>::quiet_NaN();

double seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string_view trim(std::string_view text) {
    constexpr auto space = " \t\r";
    const auto first = text.find_first_not_of(space);
    return first == text.npos ? std::string_view{} : text.substr(first, text.find_last_not_of(space) - first + 1);
}

double number(std::string_view field) {
    const std::string text(trim(field));
    char *end;
    const double value = std::strtod(text.c_str(), &end);
    if (text.empty() || end != text.c_str() + text.size() || !std::isfinite(value))
        throw std::invalid_argument("V-Sidoの数値が不正です");
    return value;
}

double joint_position(const Joint &joint, double radians) {
    return radians * joint.config.multiplier + joint.config.offset;
}

double wire_position(const Joint &joint, double position) {
    return (position - joint.config.offset) / joint.config.multiplier / rads_per_count;
}

std::pair<double, double> wire_limits(const Joint &joint) {
    const auto first = wire_position(joint, joint.config.limit_pos_min);
    const auto second = wire_position(joint, joint.config.limit_pos_max);
    return std::minmax(first, second);
}

int wire_count(const Joint &joint, double position) {
    const auto [lo, hi] = wire_limits(joint);
    return static_cast<int>(std::clamp(std::round(wire_position(joint, position)),
                                       std::ceil(lo - 1e-9), std::floor(hi + 1e-9)));
}
} // namespace

std::vector<VsidoAxisSample> decode_vsido_status(std::string_view payload) {
    constexpr std::array<std::string_view, 6> labels{"vsd", "id", "agl", "obj", "vel", "cur"};
    std::unordered_map<std::string_view, std::vector<double>> rows;
    while (!payload.empty()) {
        const auto newline = payload.find('\n');
        auto line = payload.substr(0, newline);
        payload = newline == payload.npos ? std::string_view{} : payload.substr(newline + 1);
        const auto comma = line.find(',');
        const auto label = trim(line.substr(0, comma));
        if (std::find(labels.begin(), labels.end(), label) == labels.end())
            continue;
        if (!rows.try_emplace(label).second)
            throw std::invalid_argument("V-Sidoの行ラベルが重複しています");
        if (label == "vsd")
            continue;
        auto &values = rows.at(label);
        while (line.find(',') != line.npos) {
            line.remove_prefix(line.find(',') + 1);
            values.push_back(number(line.substr(0, line.find(','))));
        }
    }
    if (!rows.count("vsd") || !rows.count("id") || !rows.count("agl") || !rows.count("obj") ||
        rows.at("id").empty() || rows.at("id").size() > 253)
        throw std::invalid_argument("V-Sidoの状態行が不足しています");
    const auto &ids = rows.at("id");
    for (const auto &[label, values] : rows)
        if (label != "vsd" && values.size() != ids.size())
            throw std::invalid_argument("V-Sidoの状態行とIDの軸数が一致しません");
    const auto optional = [&](std::string_view field, size_t i, double scale) {
        const auto row = rows.find(field);
        return row == rows.end() ? nan : row->second[i] * scale;
    };
    std::set<int> seen;
    std::vector<VsidoAxisSample> samples;
    samples.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] < 1 || ids[i] > 253 || std::trunc(ids[i]) != ids[i] || !seen.insert(int(ids[i])).second)
            throw std::invalid_argument("V-SidoのIDが不正または重複しています");
        samples.push_back({uint8_t(ids[i]), rows.at("agl")[i] * rads_per_count, rows.at("obj")[i] * rads_per_count,
                           optional("vel", i, rads_per_count * 60), optional("cur", i, .001)});
    }
    return samples;
}

VsidoSession::VsidoSession(VsidoConfig config)
    : config_(std::move(config)), joints_(make_joints(config_.joints)),
      limiter_(joints_, config_.options, [](const Joint &joint, double position) {
          return joint_position(joint, wire_count(joint, position) * rads_per_count);
      }) {
    validate_settings(joints_, config_.options);
    in_addr address{};
    if (inet_pton(AF_INET, config_.peer_address.c_str(), &address) != 1 || address.s_addr == INADDR_ANY ||
        address.s_addr == INADDR_BROADCAST)
        throw std::invalid_argument("V-Sidoのpeer_addressには機器のIPv4アドレスが必要です");
    peer_ = address.s_addr;
    if (inet_pton(AF_INET, config_.bind_address.c_str(), &address) != 1 ||
        config_.receive_port < 1 || config_.receive_port > 65535 ||
        config_.command_port < 1 || config_.command_port > 65535 ||
        !std::isfinite(config_.timeout_s) || config_.timeout_s <= 0 || config_.timeout_s > 2)
        throw std::invalid_argument("V-SidoのUDP接続設定が不正です");
    if (config_.options.configure_devices || (config_.options.torque && !config_.options.floor_enabled))
        throw std::invalid_argument("V-Sido実機指令には床保護が必要です。機器設定の書込みには対応していません");
    if (!config_.options.torque)
        return;
    if (config_.command_order.size() != joints_.size())
        throw std::invalid_argument("V-Sidoのcommand_orderを機器の軸数と送信順に合わせてください");
    std::set<std::string> names;
    for (const auto &name : config_.command_order) {
        const auto it = std::find_if(joints_.begin(), joints_.end(), [&](const auto &j) {
            return j.config.joint_name == name;
        });
        if (it == joints_.end() || !names.insert(name).second)
            throw std::invalid_argument("V-Sidoの送信関節が不明または重複しています: " + name);
        const auto [lo, hi] = wire_limits(*it);
        if (lo < -1800 - 1e-9 || hi > 1800 + 1e-9 || std::ceil(lo - 1e-9) > std::floor(hi + 1e-9))
            throw std::invalid_argument("V-Sidoの関節範囲が送信形式に収まりません: " + name);
        command_indices_.push_back(size_t(it - joints_.begin()));
    }
}

VsidoSession::~VsidoSession() { close(); }

bool VsidoSession::fail(const std::string &error) {
    error_ = error;
    failed_ = true;
    active_ = fresh_ = false;
    write_ok = 0;
    for (auto &joint : joints_) {
        joint.state.valid = 0;
        joint.state.age = nan;
    }
    return false;
}

bool VsidoSession::configure(bool passive) {
    if (socket_ >= 0)
        return fail("V-Sidoは接続済みです");
    if (!config_.hardware_verified || !config_.options.verified)
        return fail("V-Sidoのhardware_verifiedの確認が必要です");
    failed_ = false;
    passive_ = passive;
    received_at_ = -std::numeric_limits<double>::infinity();
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(config_.receive_port);
    inet_pton(AF_INET, config_.bind_address.c_str(), &local.sin_addr);
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    const int enabled = 1;
    if (socket_ < 0 || fcntl(socket_, F_SETFL, O_NONBLOCK) < 0 ||
        setsockopt(socket_, SOL_SOCKET, SO_TIMESTAMP, &enabled, sizeof(enabled)) < 0 ||
        bind(socket_, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) < 0) {
        const std::string reason = std::strerror(errno);
        close();
        return fail("V-SidoのUDP受信ポートを開けません: " + reason);
    }
    const double until = seconds() + 1.;
    while (seconds() < until && !failed_) {
        if (drain() && seconds() - received_at_ <= config_.timeout_s)
            return read();
        pollfd event{socket_, POLLIN, 0};
        poll(&event, 1, 10);
    }
    const auto reason = failed_ ? error_ : "V-Sidoの初期UDP状態を受信できません";
    close();
    return fail(reason);
}

bool VsidoSession::activate() {
    if (active_)
        return true;
    if (!read())
        return false;
    for (auto &joint : joints_)
        joint.command.position = joint.state.position;
    limiter_.reset(joints_);
    active_ = true;
    return true;
}

bool VsidoSession::deactivate() {
    active_ = false;
    write_ok = 0;
    return true;
}

void VsidoSession::close() {
    deactivate();
    fresh_ = false;
    limiter_.clear();
    if (socket_ >= 0)
        ::close(socket_);
    socket_ = -1;
}

void VsidoSession::abort() {
    fail("V-Sidoを停止しました");
    close();
}

bool VsidoSession::drain() {
    for (size_t packet = 0; packet < 32; ++packet) {
        std::array<char, 4096> buffer{};
        alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timeval))> ancillary{};
        sockaddr_in source{};
        iovec iov{buffer.data(), buffer.size()};
        msghdr message{};
        message.msg_name = &source;
        message.msg_namelen = sizeof(source);
        message.msg_iov = &iov;
        message.msg_iovlen = 1;
        message.msg_control = ancillary.data();
        message.msg_controllen = ancillary.size();
        const auto bytes = recvmsg(socket_, &message, 0);
        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            if (errno == EINTR)
                continue;
            return fail("V-SidoのUDP受信に失敗しました");
        }
        if (source.sin_addr.s_addr != peer_ || bytes == 0 || (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)))
            continue;
        double stamp = -std::numeric_limits<double>::infinity();
        for (auto *c = CMSG_FIRSTHDR(&message); c; c = CMSG_NXTHDR(&message, c)) {
            if (c->cmsg_level != SOL_SOCKET || c->cmsg_type != SCM_TIMESTAMP || c->cmsg_len < CMSG_LEN(sizeof(timeval)))
                continue;
            timeval received{};
            std::memcpy(&received, CMSG_DATA(c), sizeof(received));
            timeval now{};
            gettimeofday(&now, nullptr);
            // OSの到着時刻でキュー内に残った古いパケットを除外する。
            const double age = double(now.tv_sec - received.tv_sec) + double(now.tv_usec - received.tv_usec) * 1e-6;
            if (age >= 0 && age <= config_.timeout_s)
                stamp = seconds() - age;
            break;
        }
        if (stamp <= received_at_)
            continue;
        try {
            const auto samples = decode_vsido_status(std::string_view(buffer.data(), size_t(bytes)));
            std::array<const VsidoAxisSample *, 254> by_id{};
            for (const auto &sample : samples)
                by_id[sample.id] = &sample;
            if (!std::all_of(joints_.begin(), joints_.end(), [&](const auto &joint) {
                    const auto *sample = by_id[joint.config.id];
                    return sample && std::isfinite(joint_position(joint, sample->position)) &&
                           std::isfinite(joint_position(joint, sample->goal));
                }))
                continue;
            if (active_ && stamp - received_at_ > config_.timeout_s)
                return fail("V-SidoのUDP状態が途絶したため指令を停止しました");
            for (auto &joint : joints_) {
                const auto &sample = *by_id[joint.config.id];
                joint.state.position = joint_position(joint, sample.position);
                joint.state.goal_position_readback = joint_position(joint, sample.goal);
                joint.state.velocity = sample.velocity * joint.config.multiplier;
                joint.state.current = sample.current;
            }
            received_at_ = stamp;
        } catch (const std::invalid_argument &) {
            // 壊れたパケットで正常状態の鮮度を延長しない。
        }
    }
    return true;
}

bool VsidoSession::read() {
    if (failed_)
        return false;
    if (socket_ < 0)
        return fail("V-Sidoが未接続です");
    read_start = seconds();
    if (!drain())
        return false;
    read_end = seconds();
    const double age = read_end - received_at_;
    if (age > config_.timeout_s)
        return fail("V-SidoのUDP状態が途絶したため指令を停止しました");
    for (auto &joint : joints_) {
        joint.state.age = age;
        joint.state.valid = 1;
    }
    fresh_ = true;
    return true;
}

bool VsidoSession::write() {
    write_ok = 0;
    if (failed_)
        return false;
    if (!active_)
        return fail("V-Sidoが起動されていません");
    if (passive_ || !config_.options.torque)
        return true;
    if (!fresh_ || seconds() - received_at_ > config_.timeout_s)
        return fail("V-Sidoの新しいUDP状態がないため指令を停止しました");
    for (auto &joint : joints_) {
        auto &q = joint.command.position;
        const auto &j = joint.config;
        if (!std::isfinite(q) || (!config_.options.clip_to_limits && (q < j.limit_pos_min || q > j.limit_pos_max)))
            return fail("V-Sidoの関節位置指令が不正です: " + j.joint_name);
        q = std::clamp(q, j.limit_pos_min, j.limit_pos_max);
    }
    std::string payload;
    try {
        limiter_.prepare(joints_);
        for (size_t i : command_indices_) {
            auto &joint = joints_[i];
            const int value = wire_count(joint, joint.command.position);
            if (!payload.empty())
                payload += ',';
            payload += std::to_string(value);
            joint.command.position = joint_position(joint, value * rads_per_count);
        }
    } catch (const std::exception &error) {
        return fail(error.what());
    }
    write_start = seconds();
    if (write_start - received_at_ > config_.timeout_s)
        return fail("V-SidoのUDP状態が途絶したため指令を停止しました");
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(config_.command_port);
    peer.sin_addr.s_addr = peer_;
    const auto sent = sendto(socket_, payload.data(), payload.size(), 0,
                             reinterpret_cast<const sockaddr *>(&peer), sizeof(peer));
    write_end = seconds();
    if (sent < 0 || size_t(sent) != payload.size())
        return fail("V-SidoのUDP位置指令を送信できません");
    limiter_.commit(joints_);
    write_ok = 1;
    fresh_ = false;
    return true;
}
} // namespace fuzzrobo::topoarm_hardware
