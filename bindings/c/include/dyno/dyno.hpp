#ifndef DYNO_DYNO_HPP
#define DYNO_DYNO_HPP

#include "dyno.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyno {

class Error : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

inline void check(DynoStatus status) {
    if (status != DYNO_STATUS_OK) {
        const char* message = dyno_last_error_message();
        throw Error(message != nullptr ? message : "unknown dyno error");
    }
}

inline DynoPose identity_pose() {
    return {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
}

class Robot {
public:
    explicit Robot(const std::string& urdf_path) {
        check(dyno_robot_load_urdf(urdf_path.c_str(), &robot_));
        try {
            check(dyno_workspace_create(robot_, &workspace_));
        } catch (...) {
            dyno_robot_destroy(robot_);
            robot_ = nullptr;
            throw;
        }
    }

    ~Robot() {
        dyno_workspace_destroy(workspace_);
        dyno_robot_destroy(robot_);
    }

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    Robot(Robot&& other) noexcept
        : robot_(std::exchange(other.robot_, nullptr)),
          workspace_(std::exchange(other.workspace_, nullptr)) {}

    Robot& operator=(Robot&& other) noexcept {
        if (this != &other) {
            dyno_workspace_destroy(workspace_);
            dyno_robot_destroy(robot_);
            robot_ = std::exchange(other.robot_, nullptr);
            workspace_ = std::exchange(other.workspace_, nullptr);
        }
        return *this;
    }

    std::string name() const {
        const char* value = dyno_robot_name(robot_);
        return value != nullptr ? value : "";
    }

    std::size_t joint_count() const { return dyno_robot_joint_count(robot_); }
    std::size_t link_count() const { return dyno_robot_link_count(robot_); }

    std::size_t link_id(const std::string& name) const {
        std::size_t result = 0;
        check(dyno_robot_link_id(robot_, name.c_str(), &result));
        return result;
    }

    DynoPose forward_kinematics(
        const std::vector<double>& q, std::size_t target) {
        DynoPose result{};
        check(dyno_forward_kinematics(
            robot_, workspace_, q.data(), q.size(), target, &result));
        return result;
    }

    std::vector<double> jacobian(
        const std::vector<double>& q, std::size_t target) {
        std::vector<double> result(6 * joint_count());
        check(dyno_jacobian(robot_, workspace_, q.data(), q.size(), target,
                            result.data(), result.size()));
        return result;
    }

    std::vector<double> inverse_kinematics(
        const std::vector<double>& initial_q, std::size_t target,
        const DynoPose& desired,
        DynoIkOptions options = dyno_ik_options_default()) {
        std::vector<double> result(joint_count());
        check(dyno_inverse_kinematics(
            robot_, workspace_, initial_q.data(), initial_q.size(), target,
            &desired, options, result.data(), result.size()));
        return result;
    }

    DynoTwist forward_velocity(
        const std::vector<double>& q, const std::vector<double>& qd,
        std::size_t target, const DynoPose& base = identity_pose(),
        const DynoPose& tool = identity_pose()) {
        if (q.size() != qd.size()) {
            throw Error("q and qd must have the same length");
        }
        DynoTwist result{};
        check(dyno_forward_velocity(robot_, workspace_, q.data(), qd.data(),
                                    q.size(), target, &base, &tool, &result));
        return result;
    }

    DynoTwist forward_acceleration(
        const std::vector<double>& q, const std::vector<double>& qd,
        const std::vector<double>& qdd, std::size_t target) {
        if (q.size() != qd.size() || q.size() != qdd.size()) {
            throw Error("q, qd, and qdd must have the same length");
        }
        DynoTwist result{};
        check(dyno_forward_acceleration(
            robot_, workspace_, q.data(), qd.data(), qdd.data(), q.size(),
            target, &result));
        return result;
    }

    std::vector<double> gravity(
        const std::vector<double>& q,
        const DynoPose& base = identity_pose(),
        const std::vector<DynoLoad>& loads = {}) {
        std::vector<double> result(joint_count());
        check(dyno_gravity(robot_, workspace_, q.data(), q.size(), &base,
                           loads.data(), loads.size(), result.data(), result.size()));
        return result;
    }

    std::vector<double> inverse_dynamics(
        const std::vector<double>& q, const std::vector<double>& qd,
        const std::vector<double>& qdd,
        const DynoPose& base = identity_pose(),
        DynoTwist base_velocity = {}, DynoTwist base_acceleration = {},
        const std::vector<DynoLoad>& loads = {}) {
        if (q.size() != qd.size() || q.size() != qdd.size()) {
            throw Error("q, qd, and qdd must have the same length");
        }
        std::vector<double> result(joint_count());
        check(dyno_inverse_dynamics(
            robot_, workspace_, q.data(), qd.data(), qdd.data(), q.size(),
            &base, base_velocity, base_acceleration, loads.data(), loads.size(),
            result.data(), result.size()));
        return result;
    }

    DynoRobot* native_handle() noexcept { return robot_; }
    DynoWorkspace* workspace_handle() noexcept { return workspace_; }

private:
    DynoRobot* robot_ = nullptr;
    DynoWorkspace* workspace_ = nullptr;
};

} // namespace dyno

#endif
