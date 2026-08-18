#ifndef DYNIBO_DYNIBO_HPP
#define DYNIBO_DYNIBO_HPP

#include "dynibo.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dynibo {

class Error : public std::runtime_error {
public:
    explicit Error(const std::string& message)
        : Error(DYNIBO_STATUS_MODEL_ERROR, message) {}

    Error(DyniboStatus status, const std::string& message)
        : std::runtime_error(message), status_(status) {}

    DyniboStatus status() const noexcept { return status_; }

private:
    DyniboStatus status_;
};

inline void check(DyniboStatus status) {
    if (status != DYNIBO_STATUS_OK) {
        const char* message = dynibo_last_error_message();
        throw Error(status, message != nullptr ? message : "unknown dynibo error");
    }
}

inline DyniboPose identity_pose() {
    return {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
}

class Robot {
public:
    explicit Robot(
        const std::string& urdf_path,
        DyniboBaseMode base_mode = DYNIBO_BASE_FIXED) {
        check(dynibo_robot_load_urdf_with_base(
            urdf_path.c_str(), base_mode, &robot_));
        try {
            check(dynibo_workspace_create(robot_, &workspace_));
        } catch (...) {
            dynibo_robot_destroy(robot_);
            robot_ = nullptr;
            throw;
        }
    }

    ~Robot() {
        dynibo_workspace_destroy(workspace_);
        dynibo_robot_destroy(robot_);
    }

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    Robot(Robot&& other) noexcept
        : robot_(std::exchange(other.robot_, nullptr)),
          workspace_(std::exchange(other.workspace_, nullptr)) {}

    Robot& operator=(Robot&& other) noexcept {
        if (this != &other) {
            dynibo_workspace_destroy(workspace_);
            dynibo_robot_destroy(robot_);
            robot_ = std::exchange(other.robot_, nullptr);
            workspace_ = std::exchange(other.workspace_, nullptr);
        }
        return *this;
    }

    std::string name() const {
        const char* value = dynibo_robot_name(robot_);
        return value != nullptr ? value : "";
    }

    std::size_t joint_count() const { return dynibo_robot_joint_count(robot_); }
    std::size_t generalized_count() const {
        return dynibo_robot_generalized_count(robot_);
    }
    std::size_t link_count() const { return dynibo_robot_link_count(robot_); }

    std::size_t link_id(const std::string& name) const {
        std::size_t result = 0;
        check(dynibo_robot_link_id(robot_, name.c_str(), &result));
        return result;
    }

    DyniboPose forward_kinematics(
        const std::vector<double>& q, std::size_t target) {
        DyniboPose result{};
        check(dynibo_forward_kinematics(
            robot_, workspace_, q.data(), q.size(), target, &result));
        return result;
    }

    std::vector<double> jacobian(
        const std::vector<double>& q, std::size_t target) {
        std::vector<double> result(6 * generalized_count());
        check(dynibo_jacobian(robot_, workspace_, q.data(), q.size(), target,
                            result.data(), result.size()));
        return result;
    }

    std::vector<double> jacobian_derivative(
        const std::vector<double>& q, const std::vector<double>& qd,
        std::size_t target) {
        if (q.size() != qd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q and qd must have the same length");
        }
        std::vector<double> result(6 * generalized_count());
        check(dynibo_jacobian_derivative(
            robot_, workspace_, q.data(), qd.data(), q.size(), target,
            result.data(), result.size()));
        return result;
    }

    std::vector<double> mass_matrix(const std::vector<double>& q) {
        const std::size_t n = generalized_count();
        std::vector<double> result(n * n);
        check(dynibo_mass_matrix(
            robot_, workspace_, q.data(), q.size(), result.data(), result.size()));
        return result;
    }

    std::vector<double> coriolis_matrix(
        const std::vector<double>& q, const std::vector<double>& qd) {
        if (q.size() != qd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q and qd must have the same length");
        }
        const std::size_t n = generalized_count();
        std::vector<double> result(n * n);
        check(dynibo_coriolis_matrix(
            robot_, workspace_, q.data(), qd.data(), q.size(),
            result.data(), result.size()));
        return result;
    }

    std::vector<double> inverse_kinematics(
        const std::vector<double>& initial_q, std::size_t target,
        const DyniboPose& desired,
        DyniboIkOptions options = dynibo_ik_options_default()) {
        std::vector<double> result(joint_count());
        check(dynibo_inverse_kinematics(
            robot_, workspace_, initial_q.data(), initial_q.size(), target,
            &desired, options, result.data(), result.size()));
        return result;
    }

    DyniboTwist forward_velocity(
        const std::vector<double>& q, const std::vector<double>& qd,
        std::size_t target, const DyniboPose& base = identity_pose(),
        const DyniboPose& tool = identity_pose()) {
        if (q.size() != qd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q and qd must have the same length");
        }
        DyniboTwist result{};
        check(dynibo_forward_velocity(robot_, workspace_, q.data(), qd.data(),
                                    q.size(), target, &base, &tool, &result));
        return result;
    }

    void set_base_state(
        const DyniboPose& frame,
        DyniboTwist velocity = {},
        DyniboTwist acceleration = {}) {
        check(dynibo_robot_set_base_state(
            robot_, &frame, velocity, acceleration));
    }

    DyniboTwist forward_acceleration(
        const std::vector<double>& q, const std::vector<double>& qd,
        const std::vector<double>& qdd, std::size_t target) {
        if (q.size() != qd.size() || q.size() != qdd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q, qd, and qdd must have the same length");
        }
        DyniboTwist result{};
        check(dynibo_forward_acceleration(
            robot_, workspace_, q.data(), qd.data(), qdd.data(), q.size(),
            target, &result));
        return result;
    }

    std::vector<double> gravity(
        const std::vector<double>& q,
        const DyniboPose& base = identity_pose(),
        const std::vector<DyniboLoad>& loads = {}) {
        std::vector<double> result(generalized_count());
        check(dynibo_gravity(robot_, workspace_, q.data(), q.size(), &base,
                           loads.data(), loads.size(), result.data(), result.size()));
        return result;
    }

    std::vector<double> inverse_dynamics(
        const std::vector<double>& q, const std::vector<double>& qd,
        const std::vector<double>& qdd,
        const DyniboPose& base = identity_pose(),
        DyniboTwist base_velocity = {}, DyniboTwist base_acceleration = {},
        const std::vector<DyniboLoad>& loads = {}) {
        if (q.size() != qd.size() || q.size() != qdd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q, qd, and qdd must have the same length");
        }
        std::vector<double> result(generalized_count());
        check(dynibo_inverse_dynamics(
            robot_, workspace_, q.data(), qd.data(), qdd.data(), q.size(),
            &base, base_velocity, base_acceleration, loads.data(), loads.size(),
            result.data(), result.size()));
        return result;
    }

    DyniboRobot* native_handle() noexcept { return robot_; }
    DyniboWorkspace* workspace_handle() noexcept { return workspace_; }

private:
    DyniboRobot* robot_ = nullptr;
    DyniboWorkspace* workspace_ = nullptr;
};

} // namespace dynibo

#endif
