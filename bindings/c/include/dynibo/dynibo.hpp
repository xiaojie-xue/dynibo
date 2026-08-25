#ifndef DYNIBO_DYNIBO_HPP
#define DYNIBO_DYNIBO_HPP

/**
 * @file dynibo.hpp
 * @brief C++17 RAII interface for dynibo robot kinematics and dynamics.
 */

#include "dynibo.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dynibo {

/** @brief Exception raised when a dynibo operation fails. */
class Error : public std::runtime_error {
public:
    /** @brief Creates a model-error exception with @p message. */
    explicit Error(const std::string& message)
        : Error(DYNIBO_STATUS_MODEL_ERROR, message) {}

    /** @brief Creates an exception carrying a C API status and message. */
    Error(DyniboStatus status, const std::string& message)
        : std::runtime_error(message), status_(status) {}

    /** @brief Returns the C API status associated with this exception. */
    DyniboStatus status() const noexcept { return status_; }

private:
    DyniboStatus status_;
};

/** @brief Throws Error unless @p status indicates success. */
inline void check(DyniboStatus status) {
    if (status != DYNIBO_STATUS_OK) {
        const char* message = dynibo_last_error_message();
        throw Error(status, message != nullptr ? message : "unknown dynibo error");
    }
}

/** @brief Returns an identity pose. */
inline DyniboPose identity_pose() {
    return {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
}

/**
 * @brief A move-only robot model with an owned reusable workspace.
 *
 * Robot releases both native handles in its destructor. Calculation methods
 * mutate the owned workspace and must not be called concurrently on the same
 * object.
 */
class Robot {
public:
    /**
     * @brief Loads a robot from a URDF file.
     * @param urdf_path Path to the URDF file.
     * @param base_mode Fixed or floating root-link mode.
     * @throws Error if the model or workspace cannot be created.
     */
    explicit Robot(
        const std::string& urdf_path,
        DyniboBaseMode base_mode = DYNIBO_BASE_FIXED) {
        check(dynibo_robot_from_urdf_with_base(
            urdf_path.c_str(), base_mode, &robot_));
        try {
            check(dynibo_workspace_create(robot_, &workspace_));
        } catch (...) {
            dynibo_robot_destroy(robot_);
            robot_ = nullptr;
            throw;
        }
    }

    /** @brief Releases the owned workspace and robot handles. */
    ~Robot() {
        dynibo_workspace_destroy(workspace_);
        dynibo_robot_destroy(robot_);
    }

    /** @brief Robot objects cannot be copied. */
    Robot(const Robot&) = delete;
    /** @brief Robot objects cannot be copied. */
    Robot& operator=(const Robot&) = delete;

    /** @brief Transfers ownership from @p other. */
    Robot(Robot&& other) noexcept
        : robot_(std::exchange(other.robot_, nullptr)),
          workspace_(std::exchange(other.workspace_, nullptr)) {}

    /** @brief Releases current resources and transfers ownership from @p other. */
    Robot& operator=(Robot&& other) noexcept {
        if (this != &other) {
            dynibo_workspace_destroy(workspace_);
            dynibo_robot_destroy(robot_);
            robot_ = std::exchange(other.robot_, nullptr);
            workspace_ = std::exchange(other.workspace_, nullptr);
        }
        return *this;
    }

    /** @brief Returns the robot name declared in the URDF. */
    std::string name() const {
        const char* value = dynibo_robot_name(robot_);
        return value != nullptr ? value : "";
    }

    /** @brief Returns the number of non-fixed joints. */
    std::size_t joint_count() const { return dynibo_robot_joint_count(robot_); }
    /** @brief Returns the generalized output dimension. */
    std::size_t generalized_count() const {
        return dynibo_robot_generalized_count(robot_);
    }
    /** @brief Returns the number of links, including the root link. */
    std::size_t link_count() const { return dynibo_robot_link_count(robot_); }

    /** @brief Resolves a link name to a model-scoped ID. @throws Error on failure. */
    std::size_t link_id(const std::string& name) const {
        std::size_t result = 0;
        check(dynibo_robot_link_id(robot_, name.c_str(), &result));
        return result;
    }

    /** @brief Computes a target-link pose in the world frame. @throws Error on failure. */
    DyniboPose forward_kinematics(
        const std::vector<double>& q, std::size_t target) {
        DyniboPose result{};
        check(dynibo_forward_kinematics(
            robot_, workspace_, q.data(), q.size(), target, &result));
        return result;
    }

    /** @brief Returns a column-major geometric Jacobian of size 6 by G. */
    std::vector<double> jacobian(
        const std::vector<double>& q, std::size_t target) {
        std::vector<double> result(6 * generalized_count());
        check(dynibo_jacobian(robot_, workspace_, q.data(), q.size(), target,
                            result.data(), result.size()));
        return result;
    }

    /** @brief Returns the column-major time derivative of the geometric Jacobian. */
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

    /** @brief Returns the column-major generalized mass matrix. */
    std::vector<double> mass_matrix(const std::vector<double>& q) {
        const std::size_t n = generalized_count();
        std::vector<double> result(n * n);
        check(dynibo_mass_matrix(
            robot_, workspace_, q.data(), q.size(), result.data(), result.size()));
        return result;
    }

    /** @brief Returns Coriolis and centrifugal generalized forces. */
    std::vector<double> velocity_product_forces(
        const std::vector<double>& q, const std::vector<double>& qd) {
        if (q.size() != qd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q and qd must have the same length");
        }
        std::vector<double> result(generalized_count());
        check(dynibo_velocity_product_forces(
            robot_, workspace_, q.data(), qd.data(), q.size(),
            result.data(), result.size()));
        return result;
    }

    /** @brief Solves fixed-base inverse kinematics for one target link. */
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

    /** @brief Computes world-expressed spatial velocity at a fixed tool point. */
    DyniboTwist forward_velocity_kinematics(
        const std::vector<double>& q, const std::vector<double>& qd,
        std::size_t target, const DyniboPose& tool = identity_pose()) {
        if (q.size() != qd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q and qd must have the same length");
        }
        DyniboTwist result{};
        check(dynibo_forward_velocity_kinematics(robot_, workspace_, q.data(), qd.data(),
                                    q.size(), target, &tool, &result));
        return result;
    }

    /** @brief Replaces the root-link pose used by subsequent calculations. */
    void set_base_frame(const DyniboPose& frame) {
        check(dynibo_robot_set_base_frame(robot_, &frame));
    }

    /** @brief Replaces the pose and classical motion of a floating base. */
    void set_floating_base_state(
        const DyniboPose& frame,
        DyniboTwist velocity = {},
        DyniboTwist acceleration = {}) {
        check(dynibo_robot_set_floating_base_state(
            robot_, &frame, velocity, acceleration));
    }

    /** @brief Computes world-expressed spatial acceleration at a target link. */
    DyniboTwist forward_acceleration_kinematics(
        const std::vector<double>& q, const std::vector<double>& qd,
        const std::vector<double>& qdd, std::size_t target) {
        if (q.size() != qd.size() || q.size() != qdd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q, qd, and qdd must have the same length");
        }
        DyniboTwist result{};
        check(dynibo_forward_acceleration_kinematics(
            robot_, workspace_, q.data(), qd.data(), qdd.data(), q.size(),
            target, &result));
        return result;
    }

    /** @brief Returns gravity and external-load generalized forces. */
    std::vector<double> gravity(
        const std::vector<double>& q,
        const std::vector<DyniboLoad>& loads = {}) {
        std::vector<double> result(generalized_count());
        check(dynibo_gravity(robot_, workspace_, q.data(), q.size(),
                           loads.data(), loads.size(), result.data(), result.size()));
        return result;
    }

    /** @brief Computes Newton--Euler inverse dynamics. */
    std::vector<double> inverse_dynamics(
        const std::vector<double>& q, const std::vector<double>& qd,
        const std::vector<double>& qdd,
        const std::vector<DyniboLoad>& loads = {}) {
        if (q.size() != qd.size() || q.size() != qdd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q, qd, and qdd must have the same length");
        }
        std::vector<double> result(generalized_count());
        check(dynibo_inverse_dynamics(
            robot_, workspace_, q.data(), qd.data(), qdd.data(), q.size(),
            loads.data(), loads.size(), result.data(), result.size()));
        return result;
    }

    /** @brief Computes articulated-body forward dynamics. */
    std::vector<double> forward_dynamics(
        const std::vector<double>& q, const std::vector<double>& qd,
        const std::vector<double>& generalized_forces,
        const std::vector<DyniboLoad>& loads = {}) {
        if (q.size() != qd.size()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "q and qd must have the same length");
        }
        std::vector<double> result(generalized_count());
        check(dynibo_forward_dynamics(
            robot_, workspace_, q.data(), qd.data(), q.size(),
            generalized_forces.data(), generalized_forces.size(),
            loads.data(), loads.size(), result.data(), result.size()));
        return result;
    }

    /** @brief Returns the borrowed native robot handle for C API interoperation. */
    DyniboRobot* native_handle() noexcept { return robot_; }
    /** @brief Returns the borrowed native workspace handle for C API interoperation. */
    DyniboWorkspace* workspace_handle() noexcept { return workspace_; }

private:
    DyniboRobot* robot_ = nullptr;
    DyniboWorkspace* workspace_ = nullptr;
};

} // namespace dynibo

#endif
