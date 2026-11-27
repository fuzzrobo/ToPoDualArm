// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <vector>

namespace fuzzrobo::topoarm_hardware {
// 平面床だけを対象にした、URDFの左右TCPによる関節目標補正。ROS・MuJoCo非依存。
class FloorGuard {
  public:
    struct Result {
        std::vector<double> positions;
        std::array<double, 2> before{}, after{}, fraction{{1, 1}};
        std::array<bool, 2> recovering{{false, false}}, holding{{false, false}};
        int evaluations{0};
    };
    FloorGuard(const std::string &path, double floor_z, double margin,
               const std::string &fragment = {});
    const std::vector<std::string> &names() const { return names_; }
    std::array<double, 2> clearance(const std::vector<double> &q) const;
    Result clip(const std::vector<double> &current, const std::vector<double> &target) const;
    Result track(const std::vector<double> &measured, const std::vector<double> &previous_goal,
                 const std::vector<double> &target) const;
    double margin() const { return margin_; }

  private:
    struct Step {
        int joint{-1};
        double scale{1}, offset{0};
        Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};
        Eigen::Vector3d axis{Eigen::Vector3d::UnitX()};
    };
    std::vector<std::string> names_;
    std::array<std::vector<Step>, 2> chains_;
    std::array<Eigen::Vector3d, 2> tips_;
    std::array<std::vector<size_t>, 2> groups_;
    double floor_, margin_;
    void validate(const std::vector<double> &q) const;
    double distance(const std::vector<double> &q, int side) const;
};
} // namespace fuzzrobo::topoarm_hardware
