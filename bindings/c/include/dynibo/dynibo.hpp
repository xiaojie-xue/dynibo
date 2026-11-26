#ifndef DYNIBO_DYNIBO_HPP
#define DYNIBO_DYNIBO_HPP

/**
 * @file dynibo.hpp
 * @brief Header-only C++17 RAII wrapper over the stable dynibo C ABI.
 *
 * Conventions match dynibo.h:
 * - Units are SI (metres, radians, seconds, kilograms, newtons, newton-metres).
 * - `q`, `qd`, and `qdd` contain `joint_count()` non-fixed URDF joints in
 *   breadth-first topological order from the root link (preserving URDF joint
 *   order among children of the same parent).
 * - Fixed `Robot` has `G == J`; `FloatingRobot` has `G == J + 6` with
 *   world-frame angular (3) then linear (3) base components at the root origin
 *   prepended before the `J` joint entries.
 * - Twists represent world-frame classical velocity or classical acceleration
 *   in angular-first order `[wx, wy, wz, vx, vy, vz]`.
 * - External loads (`DyniboLoad`) are link-local resisting wrenches at the link
 *   origin (opposite sign to a physical force applied to the robot).
 * - Matrix outputs are flat column-major `std::vector<double>` values:
 *   Jacobians are `6 * G`, mass matrices are `G * G`. Use the `*_into`
 *   overloads with an exactly sized output buffer to avoid allocation.
 * - Each `Robot` or `FloatingRobot` owns one mutable workspace with no internal
 *   lock; use a separate instance per concurrent thread.
 */

#include "dynibo.h"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dynibo {
/** Exception thrown by the C++ wrapper when a C ABI call fails. */
class Error : public std::runtime_error {
public:
    explicit Error(const std::string &message) : Error(DYNIBO_STATUS_MODEL_ERROR, message) {}
    Error(DyniboStatus status, const std::string &message)
        : std::runtime_error(message), status_(status) {}
    /** Returns the underlying C ABI status code. */
    DyniboStatus status() const noexcept {
        return status_;
    }

private:
    DyniboStatus status_;
};
/** Throws dynibo::Error with the thread-local error message unless status is DYNIBO_STATUS_OK. */
inline void check(DyniboStatus status) {
    if (status != DYNIBO_STATUS_OK) {
        const char *message = dynibo_last_error_message();
        throw Error(status, message ? message : "unknown dynibo error");
    }
}
/** Returns the identity pose (zero translation, unit quaternion `(0, 0, 0, 1)`). */
inline DyniboPose identity_pose() {
    return {{0., 0., 0.}, {0., 0., 0., 1.}};
}

class Robot;
class FloatingRobot;

/** Model-scoped link identifier returned by Robot or FloatingRobot. */
class LinkId {
public:
    /** Returns the zero-based link enumeration index (0 is the root link). */
    std::size_t index() const noexcept {
        return index_;
    }
    bool operator==(const LinkId &other) const noexcept {
        return index_ == other.index_ && owner_id_ == other.owner_id_;
    }
    bool operator!=(const LinkId &other) const noexcept {
        return !(*this == other);
    }

private:
    friend class Robot;
    friend class FloatingRobot;

    LinkId(std::size_t index, std::uint64_t owner_id) noexcept
        : index_(index), owner_id_(owner_id) {}

    std::size_t index_ = 0;
    std::uint64_t owner_id_ = 0;
};

/**
 * Explicit floating-root pose, classical velocity, and classical acceleration
 * supplied to each FloatingRobot calculation. All motion values are expressed
 * in the world frame at the root-link origin.
 */
struct BaseState {
    /** Root-link pose in the world frame. */
    DyniboPose frame = identity_pose();
    /** World-frame classical velocity `[angular_xyz, linear_xyz]` at the root origin. */
    DyniboTwist velocity{};
    /** World-frame classical acceleration at the root origin (ignored by forward_dynamics). */
    DyniboTwist acceleration{};
    BaseState() = default;
    BaseState(const DyniboPose &frame_value, const DyniboTwist &velocity_value = {},
              const DyniboTwist &acceleration_value = {})
        : frame(frame_value), velocity(velocity_value), acceleration(acceleration_value) {}
    /** Converts to the ABI-compatible C DyniboBaseState struct. */
    DyniboBaseState native() const {
        return {frame, velocity, acceleration};
    }
};

namespace detail {
inline std::uint64_t next_owner_id() noexcept {
    static std::atomic<std::uint64_t> counter{1};
    return counter.fetch_add(1, std::memory_order_relaxed);
}
inline void same(const std::vector<double> &q, const std::vector<double> &values,
                 const char *name) {
    if (q.size() != values.size()) {
        throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                    std::string("q and ") + name + " must have the same length");
    }
}
inline void same3(const std::vector<double> &q, const std::vector<double> &qd,
                  const std::vector<double> &qdd) {
    same(q, qd, "qd");
    same(q, qdd, "qdd");
}
} // namespace detail

/** Fixed-base robot with an owned reusable workspace (`G == J`). Move-only; not thread-safe. */
class Robot {
public:
    /** Loads and validates a fixed-base URDF model and creates its workspace. */
    explicit Robot(const std::string &path) {
        check(dynibo_robot_from_urdf(path.c_str(), &robot_));
        try {
            check(dynibo_workspace_create(robot_, &workspace_));
        } catch (...) {
            dynibo_robot_destroy(robot_);
            robot_ = nullptr;
            throw;
        }
        owner_id_ = detail::next_owner_id();
    }
    ~Robot() {
        dynibo_workspace_destroy(workspace_);
        dynibo_robot_destroy(robot_);
    }
    Robot(const Robot &) = delete;
    Robot &operator=(const Robot &) = delete;
    Robot(Robot &&other) noexcept
        : robot_(std::exchange(other.robot_, nullptr)),
          workspace_(std::exchange(other.workspace_, nullptr)),
          owner_id_(std::exchange(other.owner_id_, 0)) {}
    Robot &operator=(Robot &&other) noexcept {
        if (this != &other) {
            dynibo_workspace_destroy(workspace_);
            dynibo_robot_destroy(robot_);
            robot_ = std::exchange(other.robot_, nullptr);
            workspace_ = std::exchange(other.workspace_, nullptr);
            owner_id_ = std::exchange(other.owner_id_, 0);
        }
        return *this;
    }
    /** Returns the robot name declared in the URDF. */
    std::string name() const {
        const char *value = dynibo_robot_name(robot_);
        return value ? value : "";
    }
    /** Returns the number of non-fixed URDF joints `J`. */
    std::size_t joint_count() const {
        return dynibo_robot_joint_count(robot_);
    }
    /** Returns the generalized coordinate count `G` (`G == J` for fixed-base robots). */
    std::size_t generalized_count() const {
        return dynibo_robot_generalized_count(robot_);
    }
    /** Returns the total number of links, including the root link. */
    std::size_t link_count() const {
        return dynibo_robot_link_count(robot_);
    }
    /** Resolves a UTF-8 URDF link name to a model-scoped LinkId. */
    LinkId link_id(const std::string &name) const {
        std::size_t value{};
        check(dynibo_robot_link_id(robot_, name.c_str(), &value));
        return LinkId(value, owner_id_);
    }
    /** Returns the model-scoped LinkId at link enumeration index (0 is the root). */
    LinkId link_id_at(std::size_t index) const {
        if (!robot_ || owner_id_ == 0 || index >= link_count()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT, "invalid link id");
        }
        return LinkId(index, owner_id_);
    }
    /**
     * Builds a DyniboLoad resisting wrench at `link`'s origin in that link's
     * local frame, after validating that `link` belongs to this robot.
     */
    DyniboLoad make_load(LinkId link, const std::array<double, 3> &torque = {0., 0., 0.},
                         const std::array<double, 3> &force = {0., 0., 0.}) const {
        DyniboLoad load{};
        load.link_id = verify_link(link);
        for (std::size_t i = 0; i < 3; ++i) {
            load.torque[i] = torque[i];
            load.force[i] = force[i];
        }
        return load;
    }
    /** Replaces the fixed root-link world pose on this robot's workspace. */
    void set_base_frame(const DyniboPose &frame) {
        check(dynibo_workspace_set_base_frame(workspace_, &frame));
    }
    /** Replaces the world-frame gravitational acceleration `[gx, gy, gz]` (m/s^2). */
    void set_gravity(const std::array<double, 3> &gravity) {
        check(dynibo_workspace_set_gravity(workspace_, gravity.data()));
    }
    /** Returns this workspace's world-frame gravitational acceleration `[gx, gy, gz]` (m/s^2). */
    std::array<double, 3> gravity_vector() const {
        std::array<double, 3> value{};
        check(dynibo_workspace_gravity(workspace_, value.data()));
        return value;
    }
    /** Writes all `link_count()` world poses (including root) into an exactly sized buffer. */
    void forward_kinematics_all_into(const std::vector<double> &q, std::vector<DyniboPose> &out) {
        check(dynibo_forward_kinematics_all(robot_, workspace_, q.data(), q.size(), out.data(),
                                            out.size()));
    }
    /** Returns all `link_count()` world poses in link-ID order, including the root. */
    std::vector<DyniboPose> forward_kinematics_all(const std::vector<double> &q) {
        std::vector<DyniboPose> out(link_count());
        forward_kinematics_all_into(q, out);
        return out;
    }
    /** Computes the world-frame pose of `target` link. */
    DyniboPose forward_kinematics(const std::vector<double> &q, LinkId target) {
        DyniboPose result{};
        check(dynibo_forward_kinematics(robot_, workspace_, q.data(), q.size(), verify_link(target),
                                        &result));
        return result;
    }
    /** Writes the column-major `6 x G` world Jacobian into a preallocated `6 * G` buffer. */
    void jacobian_into(const std::vector<double> &q, LinkId target, std::vector<double> &out) {
        check(dynibo_jacobian(robot_, workspace_, q.data(), q.size(), verify_link(target),
                              out.data(), out.size()));
    }
    /** Returns the flat column-major `6 x G` world Jacobian (angular rows before linear). */
    std::vector<double> jacobian(const std::vector<double> &q, LinkId target) {
        std::vector<double> out(6 * generalized_count());
        jacobian_into(q, target, out);
        return out;
    }
    /** Writes the column-major `6 x G` Jacobian derivative into a preallocated `6 * G` buffer. */
    void jacobian_derivative_into(const std::vector<double> &q, const std::vector<double> &qd,
                                  LinkId target, std::vector<double> &out) {
        detail::same(q, qd, "qd");
        check(dynibo_jacobian_derivative(robot_, workspace_, q.data(), qd.data(), q.size(),
                                         verify_link(target), out.data(), out.size()));
    }
    /** Returns the flat column-major `6 x G` time derivative of the geometric Jacobian. */
    std::vector<double> jacobian_derivative(const std::vector<double> &q,
                                            const std::vector<double> &qd, LinkId target) {
        std::vector<double> out(6 * generalized_count());
        jacobian_derivative_into(q, qd, target, out);
        return out;
    }
    /** Writes the symmetric column-major `G x G` mass matrix into a preallocated `G * G` buffer. */
    void mass_matrix_into(const std::vector<double> &q, std::vector<double> &out) {
        check(dynibo_mass_matrix(robot_, workspace_, q.data(), q.size(), out.data(), out.size()));
    }
    /** Returns the flat column-major `G x G` generalized inertia matrix (CRBA). */
    std::vector<double> mass_matrix(const std::vector<double> &q) {
        std::vector<double> out(generalized_count() * generalized_count());
        mass_matrix_into(q, out);
        return out;
    }
    /** Writes Coriolis/centrifugal generalized forces into a preallocated `G`-element buffer. */
    void velocity_product_forces_into(const std::vector<double> &q, const std::vector<double> &qd,
                                      std::vector<double> &out) {
        detail::same(q, qd, "qd");
        check(dynibo_velocity_product_forces(robot_, workspace_, q.data(), qd.data(), q.size(),
                                             out.data(), out.size()));
    }
    /** Returns Coriolis and centrifugal generalized forces `C(q, qd) * qd` (length `G`). */
    std::vector<double> velocity_product_forces(const std::vector<double> &q,
                                                const std::vector<double> &qd) {
        std::vector<double> out(generalized_count());
        velocity_product_forces_into(q, qd, out);
        return out;
    }
    /** Computes world-frame classical velocity of a tool point attached to `target`. */
    DyniboTwist forward_velocity_kinematics(const std::vector<double> &q,
                                            const std::vector<double> &qd, LinkId target,
                                            const DyniboPose &tool = identity_pose()) {
        detail::same(q, qd, "qd");
        DyniboTwist result{};
        check(dynibo_forward_velocity_kinematics(robot_, workspace_, q.data(), qd.data(), q.size(),
                                                 verify_link(target), &tool, &result));
        return result;
    }
    /** Computes world-frame classical acceleration at the `target` link origin. */
    DyniboTwist forward_acceleration_kinematics(const std::vector<double> &q,
                                                const std::vector<double> &qd,
                                                const std::vector<double> &qdd, LinkId target) {
        detail::same3(q, qd, qdd);
        DyniboTwist result{};
        check(dynibo_forward_acceleration_kinematics(robot_, workspace_, q.data(), qd.data(),
                                                     qdd.data(), q.size(), verify_link(target),
                                                     &result));
        return result;
    }
    /** Writes gravity (plus optional resisting loads) forces into a preallocated `G` buffer. */
    void gravity_into(const std::vector<double> &q, std::vector<double> &out,
                      const std::vector<DyniboLoad> &loads = {}) {
        check(dynibo_gravity(robot_, workspace_, q.data(), q.size(), loads.data(), loads.size(),
                             out.data(), out.size()));
    }
    /** Returns generalized gravity forces, optionally including link-local resisting loads. */
    std::vector<double> gravity(const std::vector<double> &q,
                                const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        gravity_into(q, out, loads);
        return out;
    }
    /** Writes RNEA generalized forces into a preallocated `G`-element buffer. */
    void inverse_dynamics_into(const std::vector<double> &q, const std::vector<double> &qd,
                               const std::vector<double> &qdd, std::vector<double> &out,
                               const std::vector<DyniboLoad> &loads = {}) {
        detail::same3(q, qd, qdd);
        check(dynibo_inverse_dynamics(robot_, workspace_, q.data(), qd.data(), qdd.data(), q.size(),
                                      loads.data(), loads.size(), out.data(), out.size()));
    }
    /** Returns generalized forces via recursive Newton-Euler (RNEA, length `G`). */
    std::vector<double> inverse_dynamics(const std::vector<double> &q,
                                         const std::vector<double> &qd,
                                         const std::vector<double> &qdd,
                                         const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        inverse_dynamics_into(q, qd, qdd, out, loads);
        return out;
    }
    /** Writes ABA generalized accelerations into a preallocated `G`-element buffer. */
    void forward_dynamics_into(const std::vector<double> &q, const std::vector<double> &qd,
                               const std::vector<double> &generalized_forces,
                               std::vector<double> &out,
                               const std::vector<DyniboLoad> &loads = {}) {
        detail::same(q, qd, "qd");
        check(dynibo_forward_dynamics(robot_, workspace_, q.data(), qd.data(), q.size(),
                                      generalized_forces.data(), generalized_forces.size(),
                                      loads.data(), loads.size(), out.data(), out.size()));
    }
    /** Returns generalized accelerations via the articulated-body algorithm (ABA, length `G`). */
    std::vector<double> forward_dynamics(const std::vector<double> &q,
                                         const std::vector<double> &qd,
                                         const std::vector<double> &generalized_forces,
                                         const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        forward_dynamics_into(q, qd, generalized_forces, out, loads);
        return out;
    }
    /** Solves fixed-base damped-least-squares IK into a preallocated `J`-element buffer. */
    void inverse_kinematics_into(const std::vector<double> &q, LinkId target,
                                 const DyniboPose &desired, std::vector<double> &out,
                                 DyniboIkOptions options = dynibo_ik_options_default()) {
        check(dynibo_inverse_kinematics(robot_, workspace_, q.data(), q.size(), verify_link(target),
                                        &desired, options, out.data(), out.size()));
    }
    /** Solves fixed-base damped-least-squares IK and returns `J` joint positions. */
    std::vector<double> inverse_kinematics(const std::vector<double> &q, LinkId target,
                                           const DyniboPose &desired,
                                           DyniboIkOptions options = dynibo_ik_options_default()) {
        std::vector<double> out(joint_count());
        inverse_kinematics_into(q, target, desired, out, options);
        return out;
    }
    /** Borrows the underlying C DyniboRobot handle; do not destroy or outlive this object. */
    DyniboRobot *native_handle() noexcept {
        return robot_;
    }
    /** Borrows the underlying C DyniboWorkspace handle; do not destroy or outlive this object. */
    DyniboWorkspace *workspace_handle() noexcept {
        return workspace_;
    }

private:
    std::size_t verify_link(LinkId target) const {
        if (!robot_ || owner_id_ == 0 || target.owner_id_ != owner_id_) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "link identifier does not belong to this robot model");
        }
        return target.index_;
    }

    DyniboRobot *robot_ = nullptr;
    DyniboWorkspace *workspace_ = nullptr;
    std::uint64_t owner_id_ = 0;
};

/**
 * Floating-base robot with an owned reusable workspace (`G == J + 6`).
 * BaseState is supplied explicitly to every calculation. Move-only; not thread-safe.
 */
class FloatingRobot {
public:
    /** Loads a floating-base URDF (requires positive root mass) and creates its workspace. */
    explicit FloatingRobot(const std::string &path) {
        check(dynibo_floating_robot_from_urdf(path.c_str(), &robot_));
        try {
            check(dynibo_floating_workspace_create(robot_, &workspace_));
        } catch (...) {
            dynibo_floating_robot_destroy(robot_);
            robot_ = nullptr;
            throw;
        }
        owner_id_ = detail::next_owner_id();
    }
    ~FloatingRobot() {
        dynibo_floating_workspace_destroy(workspace_);
        dynibo_floating_robot_destroy(robot_);
    }
    FloatingRobot(const FloatingRobot &) = delete;
    FloatingRobot &operator=(const FloatingRobot &) = delete;
    FloatingRobot(FloatingRobot &&other) noexcept
        : robot_(std::exchange(other.robot_, nullptr)),
          workspace_(std::exchange(other.workspace_, nullptr)),
          owner_id_(std::exchange(other.owner_id_, 0)) {}
    FloatingRobot &operator=(FloatingRobot &&other) noexcept {
        if (this != &other) {
            dynibo_floating_workspace_destroy(workspace_);
            dynibo_floating_robot_destroy(robot_);
            robot_ = std::exchange(other.robot_, nullptr);
            workspace_ = std::exchange(other.workspace_, nullptr);
            owner_id_ = std::exchange(other.owner_id_, 0);
        }
        return *this;
    }
    /** Returns the robot name declared in the URDF. */
    std::string name() const {
        const char *value = dynibo_floating_robot_name(robot_);
        return value ? value : "";
    }
    /** Returns the number of non-fixed URDF joints `J`. */
    std::size_t joint_count() const {
        return dynibo_floating_robot_joint_count(robot_);
    }
    /** Returns the generalized coordinate count `G` (`G == J + 6` for floating-base robots). */
    std::size_t generalized_count() const {
        return dynibo_floating_robot_generalized_count(robot_);
    }
    /** Returns the total number of links, including the root link. */
    std::size_t link_count() const {
        return dynibo_floating_robot_link_count(robot_);
    }
    /** Resolves a UTF-8 URDF link name to a model-scoped LinkId. */
    LinkId link_id(const std::string &name) const {
        std::size_t value{};
        check(dynibo_floating_robot_link_id(robot_, name.c_str(), &value));
        return LinkId(value, owner_id_);
    }
    /** Returns the model-scoped LinkId at link enumeration index (0 is the root). */
    LinkId link_id_at(std::size_t index) const {
        if (!robot_ || owner_id_ == 0 || index >= link_count()) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT, "invalid link id");
        }
        return LinkId(index, owner_id_);
    }
    /**
     * Builds a DyniboLoad resisting wrench at `link`'s origin in that link's
     * local frame, after validating that `link` belongs to this robot.
     */
    DyniboLoad make_load(LinkId link, const std::array<double, 3> &torque = {0., 0., 0.},
                         const std::array<double, 3> &force = {0., 0., 0.}) const {
        DyniboLoad load{};
        load.link_id = verify_link(link);
        for (std::size_t i = 0; i < 3; ++i) {
            load.torque[i] = torque[i];
            load.force[i] = force[i];
        }
        return load;
    }
    /** Replaces the world-frame gravitational acceleration `[gx, gy, gz]` (m/s^2). */
    void set_gravity(const std::array<double, 3> &gravity) {
        check(dynibo_floating_workspace_set_gravity(workspace_, gravity.data()));
    }
    /** Returns this workspace's world-frame gravitational acceleration `[gx, gy, gz]` (m/s^2). */
    std::array<double, 3> gravity_vector() const {
        std::array<double, 3> value{};
        check(dynibo_floating_workspace_gravity(workspace_, value.data()));
        return value;
    }
    /** Writes all `link_count()` world poses (including root) into an exactly sized buffer. */
    void forward_kinematics_all_into(const BaseState &base_state, const std::vector<double> &q,
                                     std::vector<DyniboPose> &out) {
        const auto base = base_state.native();
        check(dynibo_floating_forward_kinematics_all(robot_, workspace_, &base, q.data(), q.size(),
                                                     out.data(), out.size()));
    }
    /** Returns all `link_count()` world poses in link-ID order, including the root. */
    std::vector<DyniboPose> forward_kinematics_all(const BaseState &base_state,
                                                   const std::vector<double> &q) {
        std::vector<DyniboPose> out(link_count());
        forward_kinematics_all_into(base_state, q, out);
        return out;
    }
    /** Computes the world-frame pose of `target` link. */
    DyniboPose forward_kinematics(const BaseState &base_state, const std::vector<double> &q,
                                  LinkId target) {
        auto base = base_state.native();
        DyniboPose out{};
        check(dynibo_floating_forward_kinematics(robot_, workspace_, &base, q.data(), q.size(),
                                                 verify_link(target), &out));
        return out;
    }
    /** Writes the column-major `6 x G` world Jacobian into a preallocated `6 * G` buffer. */
    void jacobian_into(const BaseState &base_state, const std::vector<double> &q, LinkId target,
                       std::vector<double> &out) {
        const auto base = base_state.native();
        check(dynibo_floating_jacobian(robot_, workspace_, &base, q.data(), q.size(),
                                       verify_link(target), out.data(), out.size()));
    }
    /** Returns the flat column-major `6 x G` world Jacobian (6 base columns before `J` joints). */
    std::vector<double> jacobian(const BaseState &base_state, const std::vector<double> &q,
                                 LinkId target) {
        std::vector<double> out(6 * generalized_count());
        jacobian_into(base_state, q, target, out);
        return out;
    }
    /** Writes the column-major `6 x G` Jacobian derivative into a preallocated `6 * G` buffer. */
    void jacobian_derivative_into(const BaseState &base_state, const std::vector<double> &q,
                                  const std::vector<double> &qd, LinkId target,
                                  std::vector<double> &out) {
        detail::same(q, qd, "qd");
        const auto base = base_state.native();
        check(dynibo_floating_jacobian_derivative(robot_, workspace_, &base, q.data(), qd.data(),
                                                  q.size(), verify_link(target), out.data(),
                                                  out.size()));
    }
    /** Returns the flat column-major `6 x G` time derivative of the geometric Jacobian. */
    std::vector<double> jacobian_derivative(const BaseState &base_state,
                                            const std::vector<double> &q,
                                            const std::vector<double> &qd, LinkId target) {
        std::vector<double> out(6 * generalized_count());
        jacobian_derivative_into(base_state, q, qd, target, out);
        return out;
    }
    /** Writes the symmetric column-major `G x G` mass matrix into a preallocated `G * G` buffer. */
    void mass_matrix_into(const BaseState &base_state, const std::vector<double> &q,
                          std::vector<double> &out) {
        const auto base = base_state.native();
        check(dynibo_floating_mass_matrix(robot_, workspace_, &base, q.data(), q.size(), out.data(),
                                          out.size()));
    }
    /** Returns the flat column-major `G x G` generalized inertia matrix (CRBA). */
    std::vector<double> mass_matrix(const BaseState &base_state, const std::vector<double> &q) {
        std::vector<double> out(generalized_count() * generalized_count());
        mass_matrix_into(base_state, q, out);
        return out;
    }
    /** Writes Coriolis/centrifugal generalized forces into a preallocated `G`-element buffer. */
    void velocity_product_forces_into(const BaseState &base_state, const std::vector<double> &q,
                                      const std::vector<double> &qd, std::vector<double> &out) {
        detail::same(q, qd, "qd");
        const auto base = base_state.native();
        check(dynibo_floating_velocity_product_forces(robot_, workspace_, &base, q.data(),
                                                      qd.data(), q.size(), out.data(), out.size()));
    }
    /** Returns Coriolis and centrifugal generalized forces (root wrench then `J` joints). */
    std::vector<double> velocity_product_forces(const BaseState &base_state,
                                                const std::vector<double> &q,
                                                const std::vector<double> &qd) {
        std::vector<double> out(generalized_count());
        velocity_product_forces_into(base_state, q, qd, out);
        return out;
    }
    /** Computes world-frame classical velocity of a tool point attached to `target`. */
    DyniboTwist forward_velocity_kinematics(const BaseState &base_state,
                                            const std::vector<double> &q,
                                            const std::vector<double> &qd, LinkId target,
                                            const DyniboPose &tool = identity_pose()) {
        detail::same(q, qd, "qd");
        auto base = base_state.native();
        DyniboTwist out{};
        check(dynibo_floating_forward_velocity_kinematics(
            robot_, workspace_, &base, q.data(), qd.data(), q.size(), verify_link(target), &tool,
            &out));
        return out;
    }
    /** Computes world-frame classical acceleration at the `target` link origin. */
    DyniboTwist forward_acceleration_kinematics(const BaseState &base_state,
                                                const std::vector<double> &q,
                                                const std::vector<double> &qd,
                                                const std::vector<double> &qdd, LinkId target) {
        detail::same3(q, qd, qdd);
        auto base = base_state.native();
        DyniboTwist out{};
        check(dynibo_floating_forward_acceleration_kinematics(
            robot_, workspace_, &base, q.data(), qd.data(), qdd.data(), q.size(),
            verify_link(target), &out));
        return out;
    }
    /** Writes gravity (plus optional resisting loads) forces into a preallocated `G` buffer. */
    void gravity_into(const BaseState &base_state, const std::vector<double> &q,
                      std::vector<double> &out, const std::vector<DyniboLoad> &loads = {}) {
        const auto base = base_state.native();
        check(dynibo_floating_gravity(robot_, workspace_, &base, q.data(), q.size(), loads.data(),
                                      loads.size(), out.data(), out.size()));
    }
    /** Returns generalized gravity forces (world root wrench then `J` joint forces). */
    std::vector<double> gravity(const BaseState &base_state, const std::vector<double> &q,
                                const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        gravity_into(base_state, q, out, loads);
        return out;
    }
    /** Writes RNEA generalized forces into a preallocated `G`-element buffer. */
    void inverse_dynamics_into(const BaseState &base_state, const std::vector<double> &q,
                               const std::vector<double> &qd, const std::vector<double> &qdd,
                               std::vector<double> &out,
                               const std::vector<DyniboLoad> &loads = {}) {
        detail::same3(q, qd, qdd);
        const auto base = base_state.native();
        check(dynibo_floating_inverse_dynamics(robot_, workspace_, &base, q.data(), qd.data(),
                                               qdd.data(), q.size(), loads.data(), loads.size(),
                                               out.data(), out.size()));
    }
    /** Returns RNEA generalized forces (world root wrench then `J` joint forces). */
    std::vector<double> inverse_dynamics(const BaseState &base_state, const std::vector<double> &q,
                                         const std::vector<double> &qd,
                                         const std::vector<double> &qdd,
                                         const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        inverse_dynamics_into(base_state, q, qd, qdd, out, loads);
        return out;
    }
    /**
     * Writes ABA generalized accelerations into a preallocated `G`-element buffer.
     * Uses `base_state.frame` and `base_state.velocity`; ignores `base_state.acceleration`.
     */
    void forward_dynamics_into(const BaseState &base_state, const std::vector<double> &q,
                               const std::vector<double> &qd,
                               const std::vector<double> &generalized_forces,
                               std::vector<double> &out,
                               const std::vector<DyniboLoad> &loads = {}) {
        detail::same(q, qd, "qd");
        const auto base = base_state.native();
        check(dynibo_floating_forward_dynamics(
            robot_, workspace_, &base, q.data(), qd.data(), q.size(), generalized_forces.data(),
            generalized_forces.size(), loads.data(), loads.size(), out.data(), out.size()));
    }
    /**
     * Returns ABA generalized accelerations (world classical base acceleration then `J` joints).
     * Uses `base_state.frame` and `base_state.velocity`; ignores `base_state.acceleration`.
     */
    std::vector<double> forward_dynamics(const BaseState &base_state, const std::vector<double> &q,
                                         const std::vector<double> &qd,
                                         const std::vector<double> &generalized_forces,
                                         const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        forward_dynamics_into(base_state, q, qd, generalized_forces, out, loads);
        return out;
    }
    /** Borrows the underlying C DyniboFloatingRobot handle; do not destroy or outlive it. */
    DyniboFloatingRobot *native_handle() noexcept {
        return robot_;
    }
    /** Borrows the underlying C DyniboFloatingWorkspace handle; do not destroy or outlive it. */
    DyniboFloatingWorkspace *workspace_handle() noexcept {
        return workspace_;
    }

private:
    std::size_t verify_link(LinkId target) const {
        if (!robot_ || owner_id_ == 0 || target.owner_id_ != owner_id_) {
            throw Error(DYNIBO_STATUS_INVALID_ARGUMENT,
                        "link identifier does not belong to this robot model");
        }
        return target.index_;
    }

    DyniboFloatingRobot *robot_ = nullptr;
    DyniboFloatingWorkspace *workspace_ = nullptr;
    std::uint64_t owner_id_ = 0;
};
} // namespace dynibo
#endif
