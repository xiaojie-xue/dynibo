#ifndef DYNIBO_DYNIBO_HPP
#define DYNIBO_DYNIBO_HPP

#include "dynibo.h"
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dynibo {
class Error : public std::runtime_error { public: explicit Error(const std::string& m) : Error(DYNIBO_STATUS_MODEL_ERROR, m) {} Error(DyniboStatus s, const std::string& m) : std::runtime_error(m), status_(s) {} DyniboStatus status() const noexcept { return status_; } private: DyniboStatus status_; };
inline void check(DyniboStatus status) { if (status != DYNIBO_STATUS_OK) { const char* m = dynibo_last_error_message(); throw Error(status, m ? m : "unknown dynibo error"); } }
inline DyniboPose identity_pose() { return {{0.,0.,0.},{0.,0.,0.,1.}}; }

/** State explicitly supplied to each FloatingRobot calculation. */
struct BaseState {
    DyniboPose frame = identity_pose();
    DyniboTwist velocity{};
    DyniboTwist acceleration{};
    BaseState() = default;
    BaseState(const DyniboPose& f, const DyniboTwist& v = {}, const DyniboTwist& a = {}) : frame(f), velocity(v), acceleration(a) {}
    DyniboBaseState native() const { return {frame, velocity, acceleration}; }
};

namespace detail {
inline void same(const std::vector<double>& q, const std::vector<double>& x, const char* name) { if (q.size() != x.size()) throw Error(DYNIBO_STATUS_INVALID_ARGUMENT, std::string("q and ") + name + " must have the same length"); }
inline void same3(const std::vector<double>& q, const std::vector<double>& qd, const std::vector<double>& qdd) { same(q, qd, "qd"); same(q, qdd, "qdd"); }
}

/** Fixed-base robot with an owned reusable workspace. */
class Robot {
public:
    explicit Robot(const std::string& path) { check(dynibo_robot_from_urdf(path.c_str(), &robot_)); try { check(dynibo_workspace_create(robot_, &workspace_)); } catch (...) { dynibo_robot_destroy(robot_); robot_ = nullptr; throw; } }
    ~Robot() { dynibo_workspace_destroy(workspace_); dynibo_robot_destroy(robot_); }
    Robot(const Robot&) = delete; Robot& operator=(const Robot&) = delete;
    Robot(Robot&& other) noexcept : robot_(std::exchange(other.robot_, nullptr)), workspace_(std::exchange(other.workspace_, nullptr)) {}
    Robot& operator=(Robot&& other) noexcept { if (this != &other) { dynibo_workspace_destroy(workspace_); dynibo_robot_destroy(robot_); robot_ = std::exchange(other.robot_, nullptr); workspace_ = std::exchange(other.workspace_, nullptr); } return *this; }
    std::string name() const { const char* v = dynibo_robot_name(robot_); return v ? v : ""; }
    std::size_t joint_count() const { return dynibo_robot_joint_count(robot_); }
    std::size_t generalized_count() const { return dynibo_robot_generalized_count(robot_); }
    std::size_t link_count() const { return dynibo_robot_link_count(robot_); }
    std::size_t link_id(const std::string& name) const { std::size_t v{}; check(dynibo_robot_link_id(robot_, name.c_str(), &v)); return v; }
    void set_base_frame(const DyniboPose& frame) { check(dynibo_robot_set_base_frame(robot_, &frame)); }
    DyniboPose forward_kinematics(const std::vector<double>& q, std::size_t target) { DyniboPose x{}; check(dynibo_forward_kinematics(robot_,workspace_,q.data(),q.size(),target,&x)); return x; }
    std::vector<double> jacobian(const std::vector<double>& q, std::size_t target) { std::vector<double> x(6*generalized_count()); check(dynibo_jacobian(robot_,workspace_,q.data(),q.size(),target,x.data(),x.size())); return x; }
    std::vector<double> jacobian_derivative(const std::vector<double>& q,const std::vector<double>& qd,std::size_t target) { detail::same(q,qd,"qd"); std::vector<double>x(6*generalized_count()); check(dynibo_jacobian_derivative(robot_,workspace_,q.data(),qd.data(),q.size(),target,x.data(),x.size())); return x; }
    std::vector<double> mass_matrix(const std::vector<double>& q) { std::vector<double>x(generalized_count()*generalized_count()); check(dynibo_mass_matrix(robot_,workspace_,q.data(),q.size(),x.data(),x.size())); return x; }
    std::vector<double> velocity_product_forces(const std::vector<double>& q,const std::vector<double>& qd) { detail::same(q,qd,"qd"); std::vector<double>x(generalized_count()); check(dynibo_velocity_product_forces(robot_,workspace_,q.data(),qd.data(),q.size(),x.data(),x.size())); return x; }
    DyniboTwist forward_velocity_kinematics(const std::vector<double>& q,const std::vector<double>& qd,std::size_t target,const DyniboPose& tool=identity_pose()) { detail::same(q,qd,"qd"); DyniboTwist x{}; check(dynibo_forward_velocity_kinematics(robot_,workspace_,q.data(),qd.data(),q.size(),target,&tool,&x)); return x; }
    DyniboTwist forward_acceleration_kinematics(const std::vector<double>& q,const std::vector<double>& qd,const std::vector<double>& qdd,std::size_t target) { detail::same3(q,qd,qdd); DyniboTwist x{}; check(dynibo_forward_acceleration_kinematics(robot_,workspace_,q.data(),qd.data(),qdd.data(),q.size(),target,&x)); return x; }
    std::vector<double> gravity(const std::vector<double>& q,const std::vector<DyniboLoad>& loads={}) { std::vector<double>x(generalized_count()); check(dynibo_gravity(robot_,workspace_,q.data(),q.size(),loads.data(),loads.size(),x.data(),x.size())); return x; }
    std::vector<double> inverse_dynamics(const std::vector<double>& q,const std::vector<double>& qd,const std::vector<double>& qdd,const std::vector<DyniboLoad>& loads={}) { detail::same3(q,qd,qdd); std::vector<double>x(generalized_count()); check(dynibo_inverse_dynamics(robot_,workspace_,q.data(),qd.data(),qdd.data(),q.size(),loads.data(),loads.size(),x.data(),x.size())); return x; }
    std::vector<double> forward_dynamics(const std::vector<double>& q,const std::vector<double>& qd,const std::vector<double>& f,const std::vector<DyniboLoad>& loads={}) { detail::same(q,qd,"qd"); std::vector<double>x(generalized_count()); check(dynibo_forward_dynamics(robot_,workspace_,q.data(),qd.data(),q.size(),f.data(),f.size(),loads.data(),loads.size(),x.data(),x.size())); return x; }
    std::vector<double> inverse_kinematics(const std::vector<double>& q,std::size_t target,const DyniboPose& desired,DyniboIkOptions options=dynibo_ik_options_default()) { std::vector<double>x(joint_count()); check(dynibo_inverse_kinematics(robot_,workspace_,q.data(),q.size(),target,&desired,options,x.data(),x.size())); return x; }
    DyniboRobot* native_handle() noexcept { return robot_; } DyniboWorkspace* workspace_handle() noexcept { return workspace_; }
private: DyniboRobot* robot_ = nullptr; DyniboWorkspace* workspace_ = nullptr;
};

/** Floating-base robot. BaseState is an input, never a property of this object. */
class FloatingRobot {
public:
    explicit FloatingRobot(const std::string& path) { check(dynibo_floating_robot_from_urdf(path.c_str(), &robot_)); try { check(dynibo_floating_workspace_create(robot_, &workspace_)); } catch (...) { dynibo_floating_robot_destroy(robot_); robot_ = nullptr; throw; } }
    ~FloatingRobot() { dynibo_floating_workspace_destroy(workspace_); dynibo_floating_robot_destroy(robot_); }
    FloatingRobot(const FloatingRobot&) = delete; FloatingRobot& operator=(const FloatingRobot&) = delete;
    FloatingRobot(FloatingRobot&& other) noexcept : robot_(std::exchange(other.robot_, nullptr)), workspace_(std::exchange(other.workspace_, nullptr)) {}
    FloatingRobot& operator=(FloatingRobot&& other) noexcept { if (this != &other) { dynibo_floating_workspace_destroy(workspace_); dynibo_floating_robot_destroy(robot_); robot_ = std::exchange(other.robot_, nullptr); workspace_ = std::exchange(other.workspace_, nullptr); } return *this; }
    std::string name() const { const char* v = dynibo_floating_robot_name(robot_); return v ? v : ""; }
    std::size_t joint_count() const { return dynibo_floating_robot_joint_count(robot_); }
    std::size_t generalized_count() const { return dynibo_floating_robot_generalized_count(robot_); }
    std::size_t link_count() const { return dynibo_floating_robot_link_count(robot_); }
    std::size_t link_id(const std::string& name) const { std::size_t v{}; check(dynibo_floating_robot_link_id(robot_, name.c_str(), &v)); return v; }
    DyniboPose forward_kinematics(const BaseState& b,const std::vector<double>& q,std::size_t target) { auto x=b.native(); DyniboPose out{}; check(dynibo_floating_forward_kinematics(robot_,workspace_,&x,q.data(),q.size(),target,&out)); return out; }
    std::vector<double> jacobian(const BaseState& b,const std::vector<double>& q,std::size_t target) { auto x=b.native(); std::vector<double>out(6*generalized_count()); check(dynibo_floating_jacobian(robot_,workspace_,&x,q.data(),q.size(),target,out.data(),out.size())); return out; }
    std::vector<double> jacobian_derivative(const BaseState& b,const std::vector<double>& q,const std::vector<double>& qd,std::size_t target) { detail::same(q,qd,"qd"); auto x=b.native(); std::vector<double>out(6*generalized_count()); check(dynibo_floating_jacobian_derivative(robot_,workspace_,&x,q.data(),qd.data(),q.size(),target,out.data(),out.size())); return out; }
    std::vector<double> mass_matrix(const BaseState& b,const std::vector<double>& q) { auto x=b.native(); std::vector<double>out(generalized_count()*generalized_count()); check(dynibo_floating_mass_matrix(robot_,workspace_,&x,q.data(),q.size(),out.data(),out.size())); return out; }
    std::vector<double> velocity_product_forces(const BaseState& b,const std::vector<double>& q,const std::vector<double>& qd) { detail::same(q,qd,"qd"); auto x=b.native(); std::vector<double>out(generalized_count()); check(dynibo_floating_velocity_product_forces(robot_,workspace_,&x,q.data(),qd.data(),q.size(),out.data(),out.size())); return out; }
    DyniboTwist forward_velocity_kinematics(const BaseState& b,const std::vector<double>& q,const std::vector<double>& qd,std::size_t target,const DyniboPose& tool=identity_pose()) { detail::same(q,qd,"qd"); auto x=b.native(); DyniboTwist out{}; check(dynibo_floating_forward_velocity_kinematics(robot_,workspace_,&x,q.data(),qd.data(),q.size(),target,&tool,&out)); return out; }
    DyniboTwist forward_acceleration_kinematics(const BaseState& b,const std::vector<double>& q,const std::vector<double>& qd,const std::vector<double>& qdd,std::size_t target) { detail::same3(q,qd,qdd); auto x=b.native(); DyniboTwist out{}; check(dynibo_floating_forward_acceleration_kinematics(robot_,workspace_,&x,q.data(),qd.data(),qdd.data(),q.size(),target,&out)); return out; }
    std::vector<double> gravity(const BaseState& b,const std::vector<double>& q,const std::vector<DyniboLoad>& loads={}) { auto x=b.native(); std::vector<double>out(generalized_count()); check(dynibo_floating_gravity(robot_,workspace_,&x,q.data(),q.size(),loads.data(),loads.size(),out.data(),out.size())); return out; }
    std::vector<double> inverse_dynamics(const BaseState& b,const std::vector<double>& q,const std::vector<double>& qd,const std::vector<double>& qdd,const std::vector<DyniboLoad>& loads={}) { detail::same3(q,qd,qdd); auto x=b.native(); std::vector<double>out(generalized_count()); check(dynibo_floating_inverse_dynamics(robot_,workspace_,&x,q.data(),qd.data(),qdd.data(),q.size(),loads.data(),loads.size(),out.data(),out.size())); return out; }
    std::vector<double> forward_dynamics(const BaseState& b,const std::vector<double>& q,const std::vector<double>& qd,const std::vector<double>& f,const std::vector<DyniboLoad>& loads={}) { detail::same(q,qd,"qd"); auto x=b.native(); std::vector<double>out(generalized_count()); check(dynibo_floating_forward_dynamics(robot_,workspace_,&x,q.data(),qd.data(),q.size(),f.data(),f.size(),loads.data(),loads.size(),out.data(),out.size())); return out; }
    DyniboFloatingRobot* native_handle() noexcept { return robot_; } DyniboFloatingWorkspace* workspace_handle() noexcept { return workspace_; }
private: DyniboFloatingRobot* robot_ = nullptr; DyniboFloatingWorkspace* workspace_ = nullptr;
};
} // namespace dynibo
#endif
