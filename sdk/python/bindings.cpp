// SPDX-License-Identifier: Apache-2.0
#include "topoarm_hardware/driver.hpp"
#include "topoarm_hardware/robot_model.hpp"
#include "topoarm_hardware/target_limiter.hpp"
#include "topoarm_hardware/vsido.hpp"
#include <cmath>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tinyxml2.h>
namespace py = pybind11;
using namespace fuzzrobo::topoarm_hardware;

// 通信しない評価用の入口。SDKと同じ制限処理を呼ぶ。
class OfflineLimiter {
    std::vector<Joint> joints_;
    SessionOptions options_;
    TargetLimiter limiter_;
    bool initialized_{false}, prepared_{false};

  public:
    OfflineLimiter(const HardwareConfig &config)
        : joints_(make_joints(config.joints)), options_(config.options), limiter_(joints_, options_, nullptr) {}
    std::vector<double> prepare(const std::vector<double> &current, const std::vector<double> &target) {
        prepared_ = false;
        if (current.size() != joints_.size() || target.size() != joints_.size())
            throw std::invalid_argument("現在値・目標値の軸数が一致しません");
        for (size_t i = 0; i < joints_.size(); ++i) {
            if (!std::isfinite(current[i]) || !std::isfinite(target[i]))
                throw std::invalid_argument("関節角度が非有限値です");
            auto &d = joints_[i];
            d.state.position = current[i];
            if (!options_.clip_to_limits &&
                (target[i] < d.config.limit_pos_min || target[i] > d.config.limit_pos_max))
                throw std::invalid_argument("目標角度が可動域外です");
            d.command.position = std::clamp(target[i], d.config.limit_pos_min, d.config.limit_pos_max);
        }
        if (!initialized_) {
            limiter_.reset(joints_);
            initialized_ = true;
        }
        limiter_.prepare(joints_);
        std::vector<double> result;
        for (const auto &d : joints_)
            result.push_back(d.command.position);
        prepared_ = true;
        return result;
    }
    void commit() {
        if (!prepared_)
            throw std::runtime_error("補正済みの目標がありません");
        limiter_.commit(joints_);
        prepared_ = false;
    }
    void reset() {
        initialized_ = prepared_ = false;
        limiter_.clear();
    }
};

// 低レベルの無検証writeをPythonへ公開しない。
PYBIND11_MODULE(_native, m) {
    m.doc() = "ToPoDualArm共通SDK。ROS非依存、単位はrad/rad_s/A。";
    py::class_<NeckLimits>(m, "NeckLimits");
    py::class_<NeckEnvelopeConfig>(m, "NeckEnvelopeConfig")
        .def_readonly("pan_abs_at_outer", &NeckEnvelopeConfig::pan_abs_at_outer)
        .def_readonly("tilt_min_at_outer", &NeckEnvelopeConfig::tilt_min_at_outer)
        .def_readonly("outward_margin", &NeckEnvelopeConfig::outward_margin);
    m.def("neck_lower", &NeckGuard::lower);
    m.def("prepare_neck", &NeckGuard::prepare);
    py::class_<FloorGuard::Result>(m, "FloorResult")
        .def_readonly("positions", &FloorGuard::Result::positions)
        .def_readonly("before", &FloorGuard::Result::before)
        .def_readonly("after", &FloorGuard::Result::after)
        .def_readonly("holding", &FloorGuard::Result::holding)
        .def_readonly("fraction", &FloorGuard::Result::fraction)
        .def_readonly("recovering", &FloorGuard::Result::recovering)
        .def_readonly("evaluations", &FloorGuard::Result::evaluations);
    py::class_<FloorGuard>(m, "FloorGuard")
        .def(py::init<const std::string &, double, double>())
        .def_property_readonly("names", &FloorGuard::names)
        .def("track", &FloorGuard::track, py::call_guard<py::gil_scoped_release>())
        .def("clearance", &FloorGuard::clearance)
        .def("clip", &FloorGuard::clip, py::call_guard<py::gil_scoped_release>());
    py::class_<JointConfig>(m, "Joint")
        .def(py::init<>())
        .def_readwrite("name", &JointConfig::joint_name)
        .def_readwrite("id", &JointConfig::id)
        .def_readwrite("multiplier", &JointConfig::multiplier)
        .def_readwrite("offset", &JointConfig::offset)
        .def_readwrite("min", &JointConfig::limit_pos_min)
        .def_readwrite("max", &JointConfig::limit_pos_max)
        .def_readwrite("operating_mode", &JointConfig::operating_mode)
        .def_readwrite("current_cap", &JointConfig::current_cap)
        .def_readwrite("initial_current_limit", &JointConfig::initial_current_limit)
        .def_readwrite("profile_duration_ms", &JointConfig::profile_duration_ms)
        .def_readwrite("profile_acceleration_ms", &JointConfig::profile_acceleration_ms)
        .def_readwrite("profile_velocity", &JointConfig::profile_vel)
        .def_readwrite("profile_acceleration", &JointConfig::profile_acc)
        .def_readwrite("position_kp", &JointConfig::position_kp)
        .def_readwrite("position_ki", &JointConfig::position_ki)
        .def_readwrite("position_kd", &JointConfig::position_kd);
    py::class_<JointState>(m, "JointState")
        .def_readonly("name", &JointState::name)
        .def_readonly("model_number", &JointState::model_number)
        .def_readonly("position", &JointState::position)
        .def_readonly("velocity", &JointState::velocity)
        .def_readonly("current", &JointState::current)
        .def_readonly("pwm", &JointState::pwm)
        .def_readonly("voltage", &JointState::voltage)
        .def_readonly("temperature", &JointState::temperature)
        .def_readonly("valid", &JointState::valid)
        .def_readonly("age", &JointState::age)
        .def_readonly("torque_enabled", &JointState::torque_enabled)
        .def_readonly("hardware_error", &JointState::hardware_error)
        .def_readonly("realtime_tick", &JointState::realtime_tick)
        .def_readonly("goal", &JointState::goal_position_readback)
        .def_readonly("position_trajectory", &JointState::position_trajectory)
        .def_readonly("velocity_trajectory", &JointState::velocity_trajectory);
    py::class_<SessionOptions>(m, "Options")
        .def(py::init<>())
        .def_readwrite("read_profile", &SessionOptions::read_profile)
        .def_readwrite("xm540_p", &SessionOptions::xm540_p)
        .def_readwrite("xm540_d", &SessionOptions::xm540_d)
        .def_readwrite("xm540_profile_ms", &SessionOptions::xm540_profile_ms)
        .def_readwrite("xm540_accel_ms", &SessionOptions::xm540_accel_ms)
        .def_readwrite("xm430_p", &SessionOptions::xm430_p)
        .def_readwrite("xm430_d", &SessionOptions::xm430_d)
        .def_readwrite("xm430_profile_ms", &SessionOptions::xm430_profile_ms)
        .def_readwrite("xm430_accel_ms", &SessionOptions::xm430_accel_ms)
        .def_readwrite("neck_envelope", &SessionOptions::neck_envelope)
        .def_readwrite("neck_limits", &SessionOptions::neck_limits)
        .def_readwrite("robot_model", &SessionOptions::robot_model)
        .def_readonly("robot_model_fragment", &SessionOptions::robot_model_fragment)
        .def("robot_urdf", [](const SessionOptions &options) {
            tinyxml2::XMLDocument document;
            load_robot_urdf(document, options.robot_model, options.robot_model_fragment);
            tinyxml2::XMLPrinter printer;
            document.Print(&printer);
            return std::string(printer.CStr());
        })
        .def_readwrite("floor_enabled", &SessionOptions::floor_enabled)
        .def_readwrite("floor_z_m", &SessionOptions::floor_z_m)
        .def_readwrite("floor_margin_m", &SessionOptions::floor_margin_m)
        .def_readwrite("verified", &SessionOptions::verified)
        .def_readwrite("torque", &SessionOptions::torque)
        .def_readwrite("clip_to_limits", &SessionOptions::clip_to_limits)
        .def_readonly("apply_startup_configuration", &SessionOptions::apply_startup_configuration)
        .def_readwrite("write_pid_on_connect", &SessionOptions::write_pid_on_connect)
        .def_property_readonly("auto_reset", [](const SessionOptions &) { return SessionOptions::auto_reset; })
        .def_readwrite("return_delay_time", &SessionOptions::return_delay_time)
        .def_readwrite("retention", &SessionOptions::retention)
        .def_readwrite("configure_devices", &SessionOptions::configure_devices);
    py::class_<HardwareConfig>(m, "HardwareConfig")
        .def_readonly("neck_tilt_negative_scale", &HardwareConfig::neck_tilt_negative_scale)
        .def_property_readonly("neck_limits",
                               [](const HardwareConfig &config) {
                                   return NeckGuard::limits(make_joints(config.joints),
                                                            config.options.neck_limits);
                               })
        .def_readwrite("port", &HardwareConfig::port)
        .def_readwrite("baud_rate", &HardwareConfig::baud_rate)
        .def_readwrite("latency_timer", &HardwareConfig::latency_timer)
        .def_readwrite("hardware_verified", &HardwareConfig::hardware_verified)
        .def_readwrite("joints", &HardwareConfig::joints)
        .def_readwrite("options", &HardwareConfig::options);
    m.def("load_config", &load_config, py::arg("path"), py::arg("selected") = std::vector<std::string>{});
    m.def("decode_vsido_status", [](const py::bytes &payload) {
        py::dict result;
        for (const auto &axis : decode_vsido_status(static_cast<std::string>(payload)))
            result[py::int_(axis.id)] = py::make_tuple(axis.position, axis.goal, axis.velocity, axis.current);
        return result;
    }, py::arg("payload"));
    py::class_<OfflineLimiter>(m, "OfflineLimiter")
        .def(py::init<const HardwareConfig &>())
        .def("prepare", &OfflineLimiter::prepare)
        .def("commit", &OfflineLimiter::commit)
        .def("reset", &OfflineLimiter::reset);
    py::class_<Driver>(m, "Driver")
        .def(py::init<const std::string &, int, int, const std::vector<JointConfig> &, SessionOptions>())
        .def("connect", &Driver::connect)
        .def("open_passive", &Driver::open_passive)
        .def("enable", &Driver::enable)
        .def("disable", &Driver::disable)
        .def("close", &Driver::close)
        .def("read", &Driver::read, py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("reset_count", &Driver::reset_count)
        .def_property_readonly("last_reset_ids", &Driver::last_reset_ids)
        .def_property_readonly("reset_pending", &Driver::reset_pending)
        .def("resume_after_reset", &Driver::resume_after_reset)
        .def("command", &Driver::command)
        .def("prepare_floor_targets", &Driver::prepare_floor_targets)
        .def("floor_result", &Driver::floor_result)
        .def("command_torque_off", &Driver::command_torque_off)
        .def("verify_targets_torque_off", &Driver::verify_targets_torque_off);
}
