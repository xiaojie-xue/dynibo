#include <dynibo/dynibo.hpp>

#include <cmath>
#include <cstdlib>
#include <new>
#include <cstdio>
#include <utility>
#include <vector>

// Count C++ return-container allocation separately from the Rust/C allocator tests.
static bool count_new = false;
static std::size_t new_calls = 0;
void* operator new(std::size_t size) {
    if (count_new) ++new_calls;
    if (void* p = std::malloc(size ? size : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

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

        {
            const auto expected_jacobian = fixed.jacobian(q, target);
            std::vector<double> out_jacobian(expected_jacobian.size());
            const auto expected_jacobian_derivative = fixed.jacobian_derivative(q, qd, target);
            std::vector<double> out_jacobian_derivative(expected_jacobian_derivative.size());
            const auto expected_mass_matrix = fixed.mass_matrix(q);
            std::vector<double> out_mass_matrix(expected_mass_matrix.size());
            const auto expected_velocity_product_forces = fixed.velocity_product_forces(q, qd);
            std::vector<double> out_velocity_product_forces(expected_velocity_product_forces.size());
            const auto expected_gravity = fixed.gravity(q, loads);
            std::vector<double> out_gravity(expected_gravity.size());
            const auto expected_inverse_dynamics = fixed.inverse_dynamics(q, qd, qdd, loads);
            std::vector<double> out_inverse_dynamics(expected_inverse_dynamics.size());
            const auto expected_forward_dynamics = fixed.forward_dynamics(q, qd, fixed_forces, loads);
            std::vector<double> out_forward_dynamics(expected_forward_dynamics.size());
            const auto expected_inverse_kinematics = fixed.inverse_kinematics(q, target, pose);
            std::vector<double> out_inverse_kinematics(expected_inverse_kinematics.size());
            std::vector<DyniboPose> all_poses(fixed.link_count());
            new_calls = 0;
            count_new = true;
            for (int repeat = 0; repeat < 20; ++repeat) {
                fixed.jacobian_into(q, target, out_jacobian);
                fixed.jacobian_derivative_into(q, qd, target, out_jacobian_derivative);
                fixed.mass_matrix_into(q, out_mass_matrix);
                fixed.velocity_product_forces_into(q, qd, out_velocity_product_forces);
                fixed.gravity_into(q, out_gravity, loads);
                fixed.inverse_dynamics_into(q, qd, qdd, out_inverse_dynamics, loads);
                fixed.forward_dynamics_into(q, qd, fixed_forces, out_forward_dynamics, loads);
                fixed.inverse_kinematics_into(q, target, pose, out_inverse_kinematics);
                fixed.forward_kinematics_all_into(q, all_poses);
            }
            count_new = false;
            CHECK(new_calls == 0);
            CHECK(out_jacobian == expected_jacobian);
            CHECK(out_jacobian_derivative == expected_jacobian_derivative);
            CHECK(out_mass_matrix == expected_mass_matrix);
            CHECK(out_velocity_product_forces == expected_velocity_product_forces);
            CHECK(out_gravity == expected_gravity);
            CHECK(out_inverse_dynamics == expected_inverse_dynamics);
            CHECK(out_forward_dynamics == expected_forward_dynamics);
            CHECK(out_inverse_kinematics == expected_inverse_kinematics);
            for (std::size_t i = 0; i < all_poses.size(); ++i) {
                const auto single = fixed.forward_kinematics(q, i);
                for (int k = 0; k < 3; ++k) CHECK(std::abs(single.translation[k] - all_poses[i].translation[k]) < 1e-12);
                for (int k = 0; k < 4; ++k) CHECK(std::abs(single.rotation_xyzw[k] - all_poses[i].rotation_xyzw[k]) < 1e-12);
            }
            std::vector<double> wrong_output(1, 123.0);
            bool rejected = false;
            try { fixed.mass_matrix_into(q, wrong_output); }
            catch (const dynibo::Error& e) { rejected = e.status() == DYNIBO_STATUS_INVALID_ARGUMENT; }
            CHECK(rejected && wrong_output[0] == 123.0);
        }
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

        {
            const auto expected_jacobian = floating.jacobian(base, q, floating_target);
            std::vector<double> out_jacobian(expected_jacobian.size());
            const auto expected_jacobian_derivative = floating.jacobian_derivative(base, q, qd, floating_target);
            std::vector<double> out_jacobian_derivative(expected_jacobian_derivative.size());
            const auto expected_mass_matrix = floating.mass_matrix(base, q);
            std::vector<double> out_mass_matrix(expected_mass_matrix.size());
            const auto expected_velocity_product_forces = floating.velocity_product_forces(base, q, qd);
            std::vector<double> out_velocity_product_forces(expected_velocity_product_forces.size());
            const auto expected_gravity = floating.gravity(base, q, floating_loads);
            std::vector<double> out_gravity(expected_gravity.size());
            const auto expected_inverse_dynamics = floating.inverse_dynamics(base, q, qd, qdd, floating_loads);
            std::vector<double> out_inverse_dynamics(expected_inverse_dynamics.size());
            const auto expected_forward_dynamics = floating.forward_dynamics(base, q, qd, floating_forces, floating_loads);
            std::vector<double> out_forward_dynamics(expected_forward_dynamics.size());
            std::vector<DyniboPose> all_poses(floating.link_count());
            new_calls = 0;
            count_new = true;
            for (int repeat = 0; repeat < 20; ++repeat) {
                floating.jacobian_into(base, q, floating_target, out_jacobian);
                floating.jacobian_derivative_into(base, q, qd, floating_target, out_jacobian_derivative);
                floating.mass_matrix_into(base, q, out_mass_matrix);
                floating.velocity_product_forces_into(base, q, qd, out_velocity_product_forces);
                floating.gravity_into(base, q, out_gravity, floating_loads);
                floating.inverse_dynamics_into(base, q, qd, qdd, out_inverse_dynamics, floating_loads);
                floating.forward_dynamics_into(base, q, qd, floating_forces, out_forward_dynamics, floating_loads);
                floating.forward_kinematics_all_into(base, q, all_poses);
            }
            count_new = false;
            CHECK(new_calls == 0);
            CHECK(out_jacobian == expected_jacobian);
            CHECK(out_jacobian_derivative == expected_jacobian_derivative);
            CHECK(out_mass_matrix == expected_mass_matrix);
            CHECK(out_velocity_product_forces == expected_velocity_product_forces);
            CHECK(out_gravity == expected_gravity);
            CHECK(out_inverse_dynamics == expected_inverse_dynamics);
            CHECK(out_forward_dynamics == expected_forward_dynamics);
            for (std::size_t i = 0; i < all_poses.size(); ++i) {
                const auto single = floating.forward_kinematics(base, q, i);
                for (int k = 0; k < 3; ++k) CHECK(std::abs(single.translation[k] - all_poses[i].translation[k]) < 1e-12);
                for (int k = 0; k < 4; ++k) CHECK(std::abs(single.rotation_xyzw[k] - all_poses[i].rotation_xyzw[k]) < 1e-12);
            }
            std::vector<double> wrong_output(1, 123.0);
            bool rejected = false;
            try { floating.mass_matrix_into(base, q, wrong_output); }
            catch (const dynibo::Error& e) { rejected = e.status() == DYNIBO_STATUS_INVALID_ARGUMENT; }
            CHECK(rejected && wrong_output[0] == 123.0);
        }
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
