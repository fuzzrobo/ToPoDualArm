// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/joint_io.hpp"
#include "topoarm_hardware/target_limiter.hpp"
#include <string_view>

namespace fuzzrobo::topoarm_hardware {
struct VsidoConfig {
    std::string bind_address{"0.0.0.0"}, peer_address;
    int receive_port{8886}, command_port{8888};
    double timeout_s{.1};
    std::vector<std::string> command_order;
    std::vector<JointConfig> joints;
    bool hardware_verified{false};
    SessionOptions options;
};
VsidoConfig load_vsido_config(const std::string &path, const std::string &peer_address = {},
                              const std::string &bind_address = {});

struct VsidoAxisSample {
    uint8_t id;
    double position, goal, velocity, current;  // rad、rad、rad/s、A
};
std::vector<VsidoAxisSample> decode_vsido_status(std::string_view payload);

// 通常モードの位置指令。UDP途絶で停止し、configureからやり直すまで送信しない。
class VsidoSession final : public JointIO {
  public:
    explicit VsidoSession(VsidoConfig config);
    ~VsidoSession() override;
    VsidoSession(const VsidoSession &) = delete;
    VsidoSession &operator=(const VsidoSession &) = delete;
    bool configure(bool passive = false) override;
    bool activate() override;
    bool deactivate() override;
    void close() override;
    void abort() override;
    bool read() override;
    bool write() override;
    std::vector<Joint> &joints() override { return joints_; }

  private:
    VsidoConfig config_;
    std::vector<Joint> joints_;
    std::vector<size_t> command_indices_;
    TargetLimiter limiter_;
    int socket_{-1};
    uint32_t peer_{0};
    bool active_{false}, failed_{false}, passive_{false};
    double received_at_{-std::numeric_limits<double>::infinity()};
    bool drain();
    bool fail(const std::string &error);
};
} // namespace fuzzrobo::topoarm_hardware
