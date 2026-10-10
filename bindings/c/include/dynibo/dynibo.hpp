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
    explicit Error(const std::string &message) : Error(DYNIBO_STATUS_MODEL_ERROR, message) {}
    Error(DyniboStatus status, const std::string &message)
        : std::runtime_error(message), status_(status) {}
    DyniboStatus status() const noexcept {
        return status_;
    }

private:
    DyniboStatus status_;
};
inline void check(DyniboStatus status) {
    if (status != DYNIBO_STATUS_OK) {
        const char *message = dynibo_last_error_message();
        throw Error(status, message ? message : "unknown dynibo error");
    }
}
inline DyniboPose identity_pose() {
    return {{0., 0., 0.}, {0., 0., 0., 1.}};
}

/** State explicitly supplied to each FloatingRobot calculation. */
struct BaseState {
    DyniboPose frame = identity_pose();
    DyniboTwist velocity{};
    DyniboTwist acceleration{};
    BaseState() = default;
    BaseState(const DyniboPose &frame_value, const DyniboTwist &velocity_value = {},
              const DyniboTwist &acceleration_value = {})
        : frame(frame_value), velocity(velocity_value), acceleration(acceleration_value) {}
    DyniboBaseState native() const {
        return {frame, velocity, acceleration};
    }
};

namespace detail {
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

/** Fixed-base robot with an owned reusable workspace. */
class Robot {
public:
    explicit Robot(const std::string &path) {
        check(dynibo_robot_from_urdf(path.c_str(), &robot_));
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
    Robot(const Robot &) = delete;
    Robot &operator=(const Robot &) = delete;
    Robot(Robot &&other) noexcept
        : robot_(std::exchange(other.robot_, nullptr)),
          workspace_(std::exchange(other.workspace_, nullptr)) {}
    Robot &operator=(Robot &&other) noexcept {
        if (this != &other) {
            dynibo_workspace_destroy(workspace_);
            dynibo_robot_destroy(robot_);
            robot_ = std::exchange(other.robot_, nullptr);
            workspace_ = std::exchange(other.workspace_, nullptr);
        }
        return *this;
    }
    std::string name() const {
        const char *value = dynibo_robot_name(robot_);
        return value ? value : "";
    }
    std::size_t joint_count() const {
        return dynibo_robot_joint_count(robot_);
    }
    std::size_t generalized_count() const {
        return dynibo_robot_generalized_count(robot_);
    }
    std::size_t link_count() const {
        return dynibo_robot_link_count(robot_);
    }
    std::size_t link_id(const std::string &name) const {
        std::size_t value{};
        check(dynibo_robot_link_id(robot_, name.c_str(), &value));
        return value;
    }
    void set_base_frame(const DyniboPose &frame) {
        check(dynibo_robot_set_base_frame(robot_, &frame));
    }
    /** Writes all link poses, including the root, in link-ID order. */
    void forward_kinematics_all_into(const std::vector<double> &q, std::vector<DyniboPose> &out) {
        check(dynibo_forward_kinematics_all(robot_, workspace_, q.data(), q.size(), out.data(),
                                            out.size()));
    }
    std::vector<DyniboPose> forward_kinematics_all(const std::vector<double> &q) {
        std::vector<DyniboPose> out(link_count());
        forward_kinematics_all_into(q, out);
        return out;
    }
    DyniboPose forward_kinematics(const std::vector<double> &q, std::size_t target) {
        DyniboPose result{};
        check(dynibo_forward_kinematics(robot_, workspace_, q.data(), q.size(), target, &result));
        return result;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void jacobian_into(const std::vector<double> &q, std::size_t target, std::vector<double> &out) {
        check(dynibo_jacobian(robot_, workspace_, q.data(), q.size(), target, out.data(),
                              out.size()));
    }
    std::vector<double> jacobian(const std::vector<double> &q, std::size_t target) {
        std::vector<double> out(6 * generalized_count());
        jacobian_into(q, target, out);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void jacobian_derivative_into(const std::vector<double> &q, const std::vector<double> &qd,
                                  std::size_t target, std::vector<double> &out) {
        detail::same(q, qd, "qd");
        check(dynibo_jacobian_derivative(robot_, workspace_, q.data(), qd.data(), q.size(), target,
                                         out.data(), out.size()));
    }
    std::vector<double> jacobian_derivative(const std::vector<double> &q,
                                            const std::vector<double> &qd, std::size_t target) {
        std::vector<double> out(6 * generalized_count());
        jacobian_derivative_into(q, qd, target, out);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void mass_matrix_into(const std::vector<double> &q, std::vector<double> &out) {
        check(dynibo_mass_matrix(robot_, workspace_, q.data(), q.size(), out.data(), out.size()));
    }
    std::vector<double> mass_matrix(const std::vector<double> &q) {
        std::vector<double> out(generalized_count() * generalized_count());
        mass_matrix_into(q, out);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void velocity_product_forces_into(const std::vector<double> &q, const std::vector<double> &qd,
                                      std::vector<double> &out) {
        detail::same(q, qd, "qd");
        check(dynibo_velocity_product_forces(robot_, workspace_, q.data(), qd.data(), q.size(),
                                             out.data(), out.size()));
    }
    std::vector<double> velocity_product_forces(const std::vector<double> &q,
                                                const std::vector<double> &qd) {
        std::vector<double> out(generalized_count());
        velocity_product_forces_into(q, qd, out);
        return out;
    }
    DyniboTwist forward_velocity_kinematics(const std::vector<double> &q,
                                            const std::vector<double> &qd, std::size_t target,
                                            const DyniboPose &tool = identity_pose()) {
        detail::same(q, qd, "qd");
        DyniboTwist result{};
        check(dynibo_forward_velocity_kinematics(robot_, workspace_, q.data(), qd.data(), q.size(),
                                                 target, &tool, &result));
        return result;
    }
    DyniboTwist forward_acceleration_kinematics(const std::vector<double> &q,
                                                const std::vector<double> &qd,
                                                const std::vector<double> &qdd,
                                                std::size_t target) {
        detail::same3(q, qd, qdd);
        DyniboTwist result{};
        check(dynibo_forward_acceleration_kinematics(robot_, workspace_, q.data(), qd.data(),
                                                     qdd.data(), q.size(), target, &result));
        return result;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void gravity_into(const std::vector<double> &q, std::vector<double> &out,
                      const std::vector<DyniboLoad> &loads = {}) {
        check(dynibo_gravity(robot_, workspace_, q.data(), q.size(), loads.data(), loads.size(),
                             out.data(), out.size()));
    }
    std::vector<double> gravity(const std::vector<double> &q,
                                const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        gravity_into(q, out, loads);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void inverse_dynamics_into(const std::vector<double> &q, const std::vector<double> &qd,
                               const std::vector<double> &qdd, std::vector<double> &out,
                               const std::vector<DyniboLoad> &loads = {}) {
        detail::same3(q, qd, qdd);
        check(dynibo_inverse_dynamics(robot_, workspace_, q.data(), qd.data(), qdd.data(), q.size(),
                                      loads.data(), loads.size(), out.data(), out.size()));
    }
    std::vector<double> inverse_dynamics(const std::vector<double> &q,
                                         const std::vector<double> &qd,
                                         const std::vector<double> &qdd,
                                         const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        inverse_dynamics_into(q, qd, qdd, out, loads);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void forward_dynamics_into(const std::vector<double> &q, const std::vector<double> &qd,
                               const std::vector<double> &generalized_forces,
                               std::vector<double> &out,
                               const std::vector<DyniboLoad> &loads = {}) {
        detail::same(q, qd, "qd");
        check(dynibo_forward_dynamics(robot_, workspace_, q.data(), qd.data(), q.size(),
                                      generalized_forces.data(), generalized_forces.size(),
                                      loads.data(), loads.size(), out.data(), out.size()));
    }
    std::vector<double> forward_dynamics(const std::vector<double> &q,
                                         const std::vector<double> &qd,
                                         const std::vector<double> &generalized_forces,
                                         const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        forward_dynamics_into(q, qd, generalized_forces, out, loads);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void inverse_kinematics_into(const std::vector<double> &q, std::size_t target,
                                 const DyniboPose &desired, std::vector<double> &out,
                                 DyniboIkOptions options = dynibo_ik_options_default()) {
        check(dynibo_inverse_kinematics(robot_, workspace_, q.data(), q.size(), target, &desired,
                                        options, out.data(), out.size()));
    }
    std::vector<double> inverse_kinematics(const std::vector<double> &q, std::size_t target,
                                           const DyniboPose &desired,
                                           DyniboIkOptions options = dynibo_ik_options_default()) {
        std::vector<double> out(joint_count());
        inverse_kinematics_into(q, target, desired, out, options);
        return out;
    }
    DyniboRobot *native_handle() noexcept {
        return robot_;
    }
    DyniboWorkspace *workspace_handle() noexcept {
        return workspace_;
    }

private:
    DyniboRobot *robot_ = nullptr;
    DyniboWorkspace *workspace_ = nullptr;
};

/** Floating-base robot. BaseState is an input, never a property of this object. */
class FloatingRobot {
public:
    explicit FloatingRobot(const std::string &path) {
        check(dynibo_floating_robot_from_urdf(path.c_str(), &robot_));
        try {
            check(dynibo_floating_workspace_create(robot_, &workspace_));
        } catch (...) {
            dynibo_floating_robot_destroy(robot_);
            robot_ = nullptr;
            throw;
        }
    }
    ~FloatingRobot() {
        dynibo_floating_workspace_destroy(workspace_);
        dynibo_floating_robot_destroy(robot_);
    }
    FloatingRobot(const FloatingRobot &) = delete;
    FloatingRobot &operator=(const FloatingRobot &) = delete;
    FloatingRobot(FloatingRobot &&other) noexcept
        : robot_(std::exchange(other.robot_, nullptr)),
          workspace_(std::exchange(other.workspace_, nullptr)) {}
    FloatingRobot &operator=(FloatingRobot &&other) noexcept {
        if (this != &other) {
            dynibo_floating_workspace_destroy(workspace_);
            dynibo_floating_robot_destroy(robot_);
            robot_ = std::exchange(other.robot_, nullptr);
            workspace_ = std::exchange(other.workspace_, nullptr);
        }
        return *this;
    }
    std::string name() const {
        const char *value = dynibo_floating_robot_name(robot_);
        return value ? value : "";
    }
    std::size_t joint_count() const {
        return dynibo_floating_robot_joint_count(robot_);
    }
    std::size_t generalized_count() const {
        return dynibo_floating_robot_generalized_count(robot_);
    }
    std::size_t link_count() const {
        return dynibo_floating_robot_link_count(robot_);
    }
    std::size_t link_id(const std::string &name) const {
        std::size_t value{};
        check(dynibo_floating_robot_link_id(robot_, name.c_str(), &value));
        return value;
    }
    /** Writes all link poses, including the root, in link-ID order. */
    void forward_kinematics_all_into(const BaseState &base_state, const std::vector<double> &q,
                                     std::vector<DyniboPose> &out) {
        const auto base = base_state.native();
        check(dynibo_floating_forward_kinematics_all(robot_, workspace_, &base, q.data(), q.size(),
                                                     out.data(), out.size()));
    }
    std::vector<DyniboPose> forward_kinematics_all(const BaseState &base_state,
                                                   const std::vector<double> &q) {
        std::vector<DyniboPose> out(link_count());
        forward_kinematics_all_into(base_state, q, out);
        return out;
    }
    DyniboPose forward_kinematics(const BaseState &base_state, const std::vector<double> &q,
                                  std::size_t target) {
        auto base = base_state.native();
        DyniboPose out{};
        check(dynibo_floating_forward_kinematics(robot_, workspace_, &base, q.data(), q.size(),
                                                 target, &out));
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void jacobian_into(const BaseState &base_state, const std::vector<double> &q,
                       std::size_t target, std::vector<double> &out) {
        const auto base = base_state.native();
        check(dynibo_floating_jacobian(robot_, workspace_, &base, q.data(), q.size(), target,
                                       out.data(), out.size()));
    }
    std::vector<double> jacobian(const BaseState &base_state, const std::vector<double> &q,
                                 std::size_t target) {
        std::vector<double> out(6 * generalized_count());
        jacobian_into(base_state, q, target, out);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void jacobian_derivative_into(const BaseState &base_state, const std::vector<double> &q,
                                  const std::vector<double> &qd, std::size_t target,
                                  std::vector<double> &out) {
        detail::same(q, qd, "qd");
        const auto base = base_state.native();
        check(dynibo_floating_jacobian_derivative(robot_, workspace_, &base, q.data(), qd.data(),
                                                  q.size(), target, out.data(), out.size()));
    }
    std::vector<double> jacobian_derivative(const BaseState &base_state,
                                            const std::vector<double> &q,
                                            const std::vector<double> &qd, std::size_t target) {
        std::vector<double> out(6 * generalized_count());
        jacobian_derivative_into(base_state, q, qd, target, out);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void mass_matrix_into(const BaseState &base_state, const std::vector<double> &q,
                          std::vector<double> &out) {
        const auto base = base_state.native();
        check(dynibo_floating_mass_matrix(robot_, workspace_, &base, q.data(), q.size(), out.data(),
                                          out.size()));
    }
    std::vector<double> mass_matrix(const BaseState &base_state, const std::vector<double> &q) {
        std::vector<double> out(generalized_count() * generalized_count());
        mass_matrix_into(base_state, q, out);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void velocity_product_forces_into(const BaseState &base_state, const std::vector<double> &q,
                                      const std::vector<double> &qd, std::vector<double> &out) {
        detail::same(q, qd, "qd");
        const auto base = base_state.native();
        check(dynibo_floating_velocity_product_forces(robot_, workspace_, &base, q.data(),
                                                      qd.data(), q.size(), out.data(), out.size()));
    }
    std::vector<double> velocity_product_forces(const BaseState &base_state,
                                                const std::vector<double> &q,
                                                const std::vector<double> &qd) {
        std::vector<double> out(generalized_count());
        velocity_product_forces_into(base_state, q, qd, out);
        return out;
    }
    DyniboTwist forward_velocity_kinematics(const BaseState &base_state,
                                            const std::vector<double> &q,
                                            const std::vector<double> &qd, std::size_t target,
                                            const DyniboPose &tool = identity_pose()) {
        detail::same(q, qd, "qd");
        auto base = base_state.native();
        DyniboTwist out{};
        check(dynibo_floating_forward_velocity_kinematics(
            robot_, workspace_, &base, q.data(), qd.data(), q.size(), target, &tool, &out));
        return out;
    }
    DyniboTwist forward_acceleration_kinematics(const BaseState &base_state,
                                                const std::vector<double> &q,
                                                const std::vector<double> &qd,
                                                const std::vector<double> &qdd,
                                                std::size_t target) {
        detail::same3(q, qd, qdd);
        auto base = base_state.native();
        DyniboTwist out{};
        check(dynibo_floating_forward_acceleration_kinematics(
            robot_, workspace_, &base, q.data(), qd.data(), qdd.data(), q.size(), target, &out));
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
    void gravity_into(const BaseState &base_state, const std::vector<double> &q,
                      std::vector<double> &out, const std::vector<DyniboLoad> &loads = {}) {
        const auto base = base_state.native();
        check(dynibo_floating_gravity(robot_, workspace_, &base, q.data(), q.size(), loads.data(),
                                      loads.size(), out.data(), out.size()));
    }
    std::vector<double> gravity(const BaseState &base_state, const std::vector<double> &q,
                                const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        gravity_into(base_state, q, out, loads);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
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
    std::vector<double> inverse_dynamics(const BaseState &base_state, const std::vector<double> &q,
                                         const std::vector<double> &qd,
                                         const std::vector<double> &qdd,
                                         const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        inverse_dynamics_into(base_state, q, qd, qdd, out, loads);
        return out;
    }
    /** Writes into an exactly sized caller buffer without resizing or allocating. */
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
    std::vector<double> forward_dynamics(const BaseState &base_state, const std::vector<double> &q,
                                         const std::vector<double> &qd,
                                         const std::vector<double> &generalized_forces,
                                         const std::vector<DyniboLoad> &loads = {}) {
        std::vector<double> out(generalized_count());
        forward_dynamics_into(base_state, q, qd, generalized_forces, out, loads);
        return out;
    }
    DyniboFloatingRobot *native_handle() noexcept {
        return robot_;
    }
    DyniboFloatingWorkspace *workspace_handle() noexcept {
        return workspace_;
    }

private:
    DyniboFloatingRobot *robot_ = nullptr;
    DyniboFloatingWorkspace *workspace_ = nullptr;
};
} // namespace dynibo
#endif
