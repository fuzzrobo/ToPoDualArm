// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "topoarm_hardware/joint_types.hpp"
#include "topoarm_hardware/session_options.hpp"
namespace fuzzrobo::topoarm_hardware {
void validate_settings(const std::vector<Joint> &joints, const SessionOptions &options);
void apply_model_overrides(std::vector<Joint> &joints, const SessionOptions &options);
} // namespace fuzzrobo::topoarm_hardware
