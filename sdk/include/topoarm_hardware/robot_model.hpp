// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <string>

namespace tinyxml2 { class XMLDocument; }

namespace fuzzrobo::topoarm_hardware {
void load_robot_urdf(tinyxml2::XMLDocument &document, const std::string &base,
                     const std::string &fragment);
}
