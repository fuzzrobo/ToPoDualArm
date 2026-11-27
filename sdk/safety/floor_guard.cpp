// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/floor_guard.hpp"
#include "topoarm_hardware/robot_model.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tinyxml2.h>

namespace fuzzrobo::topoarm_hardware {
namespace {
using Eigen::Vector3d;
using Eigen::AngleAxisd;
std::string attribute(const tinyxml2::XMLElement *element, const char *name) {
    const char *value = element ? element->Attribute(name) : nullptr;
    if (!value || !*value)
        throw std::invalid_argument(std::string("URDF属性がありません: ") + name);
    return value;
}
Vector3d vector_attribute(const tinyxml2::XMLElement *element, const char *name, Vector3d fallback) {
    const char *value = element ? element->Attribute(name) : nullptr;
    if (!value)
        return fallback;
    std::istringstream in(value);
    Vector3d result;
    for (int i = 0; i < 3; ++i)
        if (!(in >> result[i]) || !std::isfinite(result[i]))
            throw std::invalid_argument(std::string("URDFベクトルが不正です: ") + name);
    std::string extra;
    if (in >> extra)
        throw std::invalid_argument("URDFベクトルの要素数が不正です");
    return result;
}
double scalar_attribute(const tinyxml2::XMLElement *element, const char *name, double fallback) {
    if (!element->Attribute(name))
        return fallback;
    double value;
    if (element->QueryDoubleAttribute(name, &value) != tinyxml2::XML_SUCCESS || !std::isfinite(value))
        throw std::invalid_argument(std::string("URDF数値が不正です: ") + name);
    return value;
}
} // namespace
FloorGuard::FloorGuard(const std::string &path, double floor_z, double margin,
                       const std::string &fragment)
    : floor_(floor_z), margin_(margin) {
    if (!std::isfinite(floor_) || !std::isfinite(margin_) || margin_ < .001)
        throw std::invalid_argument("床高さ・余裕が不正です");
    tinyxml2::XMLDocument document;
    load_robot_urdf(document, path, fragment);
    const auto *root = document.FirstChildElement("robot");
    // 入出力順は関節名で固定し、URDFのXML記述順には依存しない。
    for (const auto &side : {"R_", "L_"}) {
        for (int i = 1; i <= 7; ++i)
            names_.push_back(std::string(side) + "joint" + std::to_string(i));
        names_.push_back(std::string(side) + "gripper_joint");
    }
    std::set<std::string> links, joint_names, independent;
    std::map<std::string, const tinyxml2::XMLElement *> parents;
    std::map<std::string, std::vector<std::string>> children;
    for (auto *link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
        if (!links.insert(attribute(link, "name")).second)
            throw std::invalid_argument("URDFのlink名が重複しています");
    }
    for (auto *joint = root->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
        const auto name = attribute(joint, "name"), type = attribute(joint, "type");
        const auto parent = attribute(joint->FirstChildElement("parent"), "link");
        const auto child = attribute(joint->FirstChildElement("child"), "link");
        if (!joint_names.insert(name).second || !links.count(parent) || !links.count(child) ||
            !parents.emplace(child, joint).second)
            throw std::invalid_argument("URDFの関節木が不正です: " + name);
        if (type != "fixed" && type != "revolute" && type != "continuous")
            throw std::invalid_argument("未対応のURDF関節種別です: " + type);
        if (type != "fixed" && !joint->FirstChildElement("mimic"))
            independent.insert(name);
        children[parent].push_back(child);
    }
    const bool pan = independent.count("neck_pan_joint");
    const bool tilt = independent.count("neck_tilt_joint");
    if (pan != tilt)
        throw std::invalid_argument("URDFのパンチルトには両軸が必要です");
    if (pan)
        names_.insert(names_.end(), {"neck_pan_joint", "neck_tilt_joint"});
    if (independent != std::set<std::string>(names_.begin(), names_.end()))
        throw std::invalid_argument("URDFには両腕・指の16関節、またはパンチルト付きの18関節が必要です");
    std::map<std::string, int> indices;
    for (size_t i = 0; i < names_.size(); ++i)
        indices[names_[i]] = int(i);
    std::vector<std::string> roots;
    for (const auto &link : links)
        if (!parents.count(link))
            roots.push_back(link);
    if (roots.size() != 1)
        throw std::invalid_argument("URDFのroot linkが一意ではありません");
    std::map<std::string, int> body_indices;
    struct Body { int parent; Step step; };
    std::vector<Body> bodies;
    std::function<void(const std::string &, int)> append = [&](const std::string &name, int parent) {
        if (body_indices.count(name))
            throw std::invalid_argument("URDFの関節木に循環があります");
        Step step;
        if (parents.count(name)) {
            const auto *joint = parents.at(name);
            const auto *origin = joint->FirstChildElement("origin");
            step.origin.translation() = vector_attribute(origin, "xyz", Vector3d::Zero());
            const auto rpy = vector_attribute(origin, "rpy", {0, 0, 0});
            step.origin.linear() = (AngleAxisd(rpy[2], Vector3d::UnitZ()) *
                AngleAxisd(rpy[1], Vector3d::UnitY()) * AngleAxisd(rpy[0], Vector3d::UnitX())).toRotationMatrix();
            if (attribute(joint, "type") != "fixed") {
                step.axis = vector_attribute(joint->FirstChildElement("axis"), "xyz", Vector3d::UnitX());
                const double norm = step.axis.norm();
                if (!std::isfinite(norm) || norm < 1e-12)
                    throw std::invalid_argument("URDFの回転軸が不正です");
                step.axis /= norm;
                std::string source = attribute(joint, "name");
                if (const auto *mimic = joint->FirstChildElement("mimic")) {
                    source = attribute(mimic, "joint");
                    step.scale = scalar_attribute(mimic, "multiplier", 1);
                    step.offset = scalar_attribute(mimic, "offset", 0);
                }
                if (!indices.count(source) || step.scale == 0)
                    throw std::invalid_argument("URDFのmimic参照が不正です: " + source);
                step.joint = indices.at(source);
            }
        }
        const int index = int(bodies.size());
        body_indices[name] = index;
        bodies.push_back({parent, step});
        for (const auto &child : children[name])
            append(child, index);
    };
    append(roots.front(), -1);
    if (body_indices.size() != links.size())
        throw std::invalid_argument("URDFに到達不能なlinkがあります");
    std::set<size_t> used;
    for (int side = 0; side < 2; ++side) {
        const std::string tcp = side == 0 ? "R_tcp" : "L_tcp";
        if (!body_indices.count(tcp))
            throw std::invalid_argument("URDFに手先基準点がありません: " + tcp);
        std::vector<int> path;
        for (int i = body_indices.at(tcp); i >= 0; i = bodies[i].parent) {
            path.push_back(i);
            if (bodies[i].step.joint < 0)
                continue;
            const auto index = size_t(bodies[i].step.joint);
            if (!used.insert(index).second)
                throw std::invalid_argument("左右TCPの共通可動関節・連動経路は未対応です");
            groups_[side].push_back(index);
        }
        if (groups_[side].empty())
            throw std::invalid_argument("TCPに可動関節がありません");
        // 固定リンクを畳み込み、実行時にはTCP経路の可動関節だけを評価する。
        Eigen::Isometry3d fixed = Eigen::Isometry3d::Identity();
        for (auto it = path.rbegin(); it != path.rend(); ++it) {
            auto step = bodies[*it].step;
            fixed = fixed * step.origin;
            if (step.joint >= 0) {
                step.origin = fixed;
                chains_[side].push_back(step);
                fixed.setIdentity();
            }
        }
        tips_[side] = fixed.translation();
    }
}
void FloorGuard::validate(const std::vector<double> &q) const {
    if (q.size() != names_.size())
        throw std::invalid_argument("床補正の軸数が不一致です");
    for (double v : q)
        if (!std::isfinite(v))
            throw std::invalid_argument("床補正の角度が非有限です");
}
double FloorGuard::distance(const std::vector<double> &q, int side) const {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (const auto &step : chains_[side]) {
        pose = pose * step.origin;
        pose.rotate(AngleAxisd(q[step.joint] * step.scale + step.offset, step.axis));
    }
    // メッシュや肘は判定しない。URDFに明示された左右TCP原点の高さだけを使う。
    const double d = (pose * tips_[side]).z() - floor_;
    if (!std::isfinite(d))
        throw std::runtime_error("床距離を計算できません");
    return d;
}
std::array<double, 2> FloorGuard::clearance(const std::vector<double> &q) const {
    validate(q);
    return {distance(q, 0), distance(q, 1)};
}
FloorGuard::Result FloorGuard::clip(const std::vector<double> &current,
                                    const std::vector<double> &target) const {
    validate(current);
    validate(target);
    Result result;
    result.positions = target;
    result.before = clearance(target);
    result.evaluations = 2;
    // 量子化・微小な追従誤差に備え、目標には要求余裕に2mmを加える。
    const double stop_margin = margin_ + .002;
    for (int side = 0; side < 2; ++side) {
        auto q = current;
        double span = 0;
        for (size_t j : groups_[side])
            span = std::max(span, std::abs(target[j] - current[j]));
        const double initial = distance(current, side);
        ++result.evaluations;
        result.recovering[side] = initial < stop_margin;
        // 余裕内の実測は停止状態を固定せず、床から離れる経路だけ許す。
        double threshold = std::min(initial, stop_margin), previous = 0;
        int count = std::max(1, int(std::ceil(span / (.25 * 3.14159265358979323846 / 180))));
        if (count > 10000)
            throw std::invalid_argument("床補正の入力角度が過大です");
        auto evaluate = [&](double t) {
            for (size_t j : groups_[side])
                q[j] = current[j] + t * (target[j] - current[j]);
            ++result.evaluations;
            return distance(q, side);
        };
        for (int k = 1; k <= count; ++k) {
            double t = double(k) / count, d = evaluate(t);
            if (d + 1e-10 < threshold) {
                double lo = previous, hi = t;
                for (int n = 0; n < 16; ++n) {
                    double mid = (lo + hi) / 2;
                    if (evaluate(mid) + 1e-10 >= threshold)
                        lo = mid;
                    else
                        hi = mid;
                }
                result.fraction[side] = lo;
                break;
            }
            previous = t;
            threshold = std::min(stop_margin, std::max(threshold, d));
        }
        for (size_t j : groups_[side])
            result.positions[j] = current[j] + result.fraction[side] * (target[j] - current[j]);
    }
    result.after = clearance(result.positions);
    result.evaluations += 2;
    return result;
}
FloorGuard::Result FloorGuard::track(const std::vector<double> &measured,
                                     const std::vector<double> &previous_goal,
                                     const std::vector<double> &target) const {
    // 補正の起点は最後に送信できた目標。荷重で動く実測値を保持目標に取り込まない。
    validate(measured);
    validate(previous_goal);
    validate(target);
    auto result = clip(previous_goal, target);
    auto reachable = clip(measured, result.positions);
    result.evaluations += reachable.evaluations;
    for (int side = 0; side < 2; ++side) {
        result.recovering[side] = reachable.recovering[side];
        if (result.fraction[side] < 1e-7 || reachable.fraction[side] < 1 - 1e-7) {
            result.holding[side] = true;
            result.fraction[side] = 0;
            for (size_t j : groups_[side])
                result.positions[j] = previous_goal[j];
        }
    }
    result.after = clearance(result.positions);
    result.evaluations += 2;
    return result;
}

} // namespace fuzzrobo::topoarm_hardware
