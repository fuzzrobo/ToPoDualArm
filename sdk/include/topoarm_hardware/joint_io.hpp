// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/joint_types.hpp"

namespace fuzzrobo::topoarm_hardware {
// 関節I/Oのライフサイクル。公開した関節・計測のアドレスは接続中も変えない。
class JointIO {
  public:
    virtual ~JointIO() = default;
    virtual bool configure(bool passive = false) = 0;
    virtual bool activate() = 0;
    virtual bool deactivate() = 0;
    virtual void close() = 0;
    virtual void abort() = 0;
    virtual bool read() = 0;
    virtual bool write() = 0;
    virtual std::vector<Joint> &joints() = 0;
    bool fresh() const { return fresh_; }
    const std::string &error() const { return error_; }
    double read_start{0}, read_end{0}, write_start{0}, write_end{0}, write_ok{0};

  protected:
    bool fresh_{false};
    std::string error_;
};
} // namespace fuzzrobo::topoarm_hardware
