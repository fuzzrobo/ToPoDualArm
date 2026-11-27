// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/robot_model.hpp"
#include <stdexcept>
#include <tinyxml2.h>

namespace fuzzrobo::topoarm_hardware {
namespace {
tinyxml2::XMLElement &read_robot(tinyxml2::XMLDocument &document, const std::string &path) {
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS || !document.FirstChildElement("robot"))
        throw std::invalid_argument("URDFのrobotを読み込めません: " + path);
    return *document.FirstChildElement("robot");
}
}

void load_robot_urdf(tinyxml2::XMLDocument &document, const std::string &base,
                     const std::string &fragment) {
    auto &robot = read_robot(document, base);
    if (fragment.empty())
        return;
    tinyxml2::XMLDocument added;
    const auto &part = read_robot(added, fragment);
    for (auto *element = part.FirstChildElement(); element; element = element->NextSiblingElement())
        robot.InsertEndChild(element->DeepClone(&document));
}
}
