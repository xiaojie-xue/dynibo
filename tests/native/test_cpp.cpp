#include <dynibo/dynibo.hpp>

#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "failed: %s\n", #x); return 1; } } while (0)

int main(int argc, char **argv) {
    CHECK(argc >= 2);
    try {
        dynibo::Robot fixed(argv[1]);
        const auto target = fixed.link_id("test_link_4");
        const std::vector<double> q{0.2, 1.0, -0.7, 0.4};
        const std::vector<double> qd{-0.3, 0.5, -0.2, 0.8};
        const std::vector<double> qdd{0.7, -0.4, 0.1, 0.3};
        DyniboLoad load{};
        load.link_id = target;
        load.force[1] = 1.0;
        const std::vector<DyniboLoad> loads{load};

        const auto pose = fixed.forward_kinematics(q, target);
        CHECK(fixed.jacobian(q, target).size() == 6 * fixed.generalized_count());
        CHECK(fixed.jacobian_derivative(q, qd, target).size() == 6 * fixed.generalized_count());
        CHECK(fixed.mass_matrix(q).size() == fixed.generalized_count() * fixed.generalized_count());
        CHECK(fixed.velocity_product_forces(q, qd).size() == fixed.generalized_count());
        CHECK(fixed.forward_velocity_kinematics(q, qd, target).angular[0] == fixed.forward_velocity_kinematics(q, qd, target).angular[0]);
        CHECK(fixed.forward_acceleration_kinematics(q, qd, qdd, target).linear[0] == fixed.forward_acceleration_kinematics(q, qd, qdd, target).linear[0]);
        CHECK(fixed.gravity(q, loads).size() == fixed.generalized_count());
        const auto fixed_forces = fixed.inverse_dynamics(q, qd, qdd, loads);
        CHECK(fixed_forces.size() == fixed.generalized_count());
        CHECK(fixed.forward_dynamics(q, qd, fixed_forces, loads).size() == fixed.generalized_count());
        CHECK(fixed.inverse_kinematics(q, target, pose).size() == fixed.joint_count());
        DyniboPose shift = dynibo::identity_pose();
        shift.translation[0] = 0.2;
        fixed.set_base_frame(shift);
        CHECK(std::abs(fixed.forward_kinematics(q, target).translation[0] - pose.translation[0] - 0.2) < 1e-12);

        const std::vector<double> short_q{0.0, 0.0, 0.0};
        bool invalid_length = false;
        try { static_cast<void>(fixed.jacobian(short_q, target)); } catch (const dynibo::Error& error) { invalid_length = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT; }
        CHECK(invalid_length);

        dynibo::FloatingRobot floating(argv[1]);
        const auto floating_target = floating.link_id("test_link_4");
        dynibo::BaseState base;
        base.velocity.angular[0] = 0.1;
        base.acceleration.linear[1] = -0.2;
        CHECK(floating.generalized_count() == floating.joint_count() + 6);
        const auto floating_pose = floating.forward_kinematics(base, q, floating_target);
        CHECK(floating.jacobian(base, q, floating_target).size() == 6 * floating.generalized_count());
        CHECK(floating.jacobian_derivative(base, q, qd, floating_target).size() == 6 * floating.generalized_count());
        CHECK(floating.mass_matrix(base, q).size() == floating.generalized_count() * floating.generalized_count());
        CHECK(floating.velocity_product_forces(base, q, qd).size() == floating.generalized_count());
        CHECK(floating.forward_velocity_kinematics(base, q, qd, floating_target).angular[0] == floating.forward_velocity_kinematics(base, q, qd, floating_target).angular[0]);
        CHECK(floating.forward_acceleration_kinematics(base, q, qd, qdd, floating_target).linear[0] == floating.forward_acceleration_kinematics(base, q, qd, qdd, floating_target).linear[0]);
        load.link_id = floating_target;
        const std::vector<DyniboLoad> floating_loads{load};
        CHECK(floating.gravity(base, q, floating_loads).size() == floating.generalized_count());
        const auto floating_forces = floating.inverse_dynamics(base, q, qd, qdd, floating_loads);
        CHECK(floating_forces.size() == floating.generalized_count());
        CHECK(floating.forward_dynamics(base, q, qd, floating_forces, floating_loads).size() == floating.generalized_count());
        dynibo::BaseState moved_base = base;
        moved_base.frame.translation[0] = 0.3;
        static_cast<void>(floating.forward_kinematics(moved_base, q, floating_target));
        CHECK(floating.forward_kinematics(base, q, floating_target).translation[0] == floating_pose.translation[0]);
        invalid_length = false;
        try { static_cast<void>(floating.mass_matrix(base, short_q)); } catch (const dynibo::Error& error) { invalid_length = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT; }
        CHECK(invalid_length);

        dynibo::Robot moved_fixed(std::move(fixed));
        dynibo::Robot assigned_fixed(argv[1]);
        assigned_fixed = std::move(moved_fixed);
        CHECK(assigned_fixed.joint_count() == q.size());
        dynibo::FloatingRobot moved_floating(std::move(floating));
        dynibo::FloatingRobot assigned_floating(argv[1]);
        assigned_floating = std::move(moved_floating);
        CHECK(assigned_floating.generalized_count() == q.size() + 6);
    } catch (const dynibo::Error& error) {
        std::fprintf(stderr, "dynibo error: %s\n", error.what());
        return 1;
    }
    return 0;
}
