#include <dynibo/dynibo.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#define CHECK(expression)                                                       \
    do {                                                                        \
        if (!(expression)) {                                                    \
            std::cerr << "check failed at " << __FILE__ << ':' << __LINE__      \
                      << ": " << #expression << '\n';                          \
            return 1;                                                           \
        }                                                                       \
    } while (false)

static std::vector<double> read_reference(
    const char* path, const std::string& key, std::size_t count) {
    std::ifstream input(path);
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#') continue;
        std::istringstream fields(line);
        std::string field;
        std::getline(fields, field, '\t');
        if (field != key) continue;
        std::vector<double> values;
        while (std::getline(fields, field, '\t')) values.push_back(std::stod(field));
        if (values.size() == count) return values;
        return {};
    }
    return {};
}

int main(int argc, char** argv) {
    CHECK(argc == 3);
    dynibo::Robot robot(argv[1]);
    dynibo::Robot moved(std::move(robot));
    dynibo::Robot assigned(argv[1]);
    assigned = std::move(moved);
    CHECK(assigned.native_handle() != nullptr);
    CHECK(assigned.workspace_handle() != nullptr);
    CHECK(assigned.name() == "test_arm");
    CHECK(assigned.joint_count() == 4);
    CHECK(assigned.link_count() == 5);

    const auto target = assigned.link_id("test_link_4");
    const std::vector<double> q(assigned.joint_count(), 0.0);
    const auto pose = assigned.forward_kinematics(q, target);
    CHECK(std::abs(pose.translation[0] - 0.62) < 1.0e-12);
    CHECK(assigned.jacobian(q, target).size() == 6 * assigned.joint_count());
    const auto gravity = assigned.gravity(q);
    CHECK(gravity.size() == assigned.joint_count());
    CHECK(assigned.inverse_dynamics(q, q, q).size() == assigned.joint_count());
    CHECK(assigned.inverse_kinematics(q, target, pose) == q);

    const auto reference_q = read_reference(argv[2], "q", 4);
    const auto reference_qd = read_reference(argv[2], "qd", 4);
    const auto reference_qdd = read_reference(argv[2], "qdd", 4);
    const auto expected_translation = read_reference(argv[2], "fk_translation", 3);
    const auto expected_gravity = read_reference(argv[2], "gravity", 4);
    const auto expected_dynamics = read_reference(argv[2], "rnea", 4);
    CHECK(reference_q.size() == 4 && reference_qd.size() == 4);
    CHECK(reference_qdd.size() == 4 && expected_translation.size() == 3);
    CHECK(expected_gravity.size() == 4 && expected_dynamics.size() == 4);
    const auto reference_pose = assigned.forward_kinematics(reference_q, target);
    for (std::size_t index = 0; index < expected_translation.size(); ++index) {
        CHECK(std::abs(reference_pose.translation[index] - expected_translation[index]) < 2.0e-12);
    }
    const auto reference_gravity = assigned.gravity(reference_q);
    const auto reference_dynamics =
        assigned.inverse_dynamics(reference_q, reference_qd, reference_qdd);
    for (std::size_t index = 0; index < reference_q.size(); ++index) {
        CHECK(std::abs(reference_gravity[index] - expected_gravity[index]) < 2.0e-10);
        CHECK(std::abs(reference_dynamics[index] - expected_dynamics[index]) < 2.0e-10);
    }
    const auto recovered_acceleration =
        assigned.forward_dynamics(reference_q, reference_qd, reference_dynamics);
    for (std::size_t index = 0; index < reference_q.size(); ++index) {
        CHECK(std::abs(recovered_acceleration[index] - reference_qdd[index]) < 2.0e-10);
    }

    const auto mass = assigned.mass_matrix(reference_q);
    CHECK(mass.size() == reference_q.size() * reference_q.size());
    for (std::size_t row = 0; row < reference_q.size(); ++row) {
        for (std::size_t column = 0; column < reference_q.size(); ++column) {
            CHECK(std::abs(mass[column * 4 + row] - mass[row * 4 + column]) < 1.0e-12);
        }
    }
    const auto velocity_product =
        assigned.velocity_product_forces(reference_q, reference_qd);
    CHECK(velocity_product.size() == reference_q.size());
    const std::vector<double> zero_qdd(reference_q.size(), 0.0);
    const auto bias = assigned.inverse_dynamics(reference_q, reference_qd, zero_qdd);
    for (std::size_t row = 0; row < reference_q.size(); ++row) {
        const double reconstructed = reference_gravity[row] + velocity_product[row];
        CHECK(std::abs(reconstructed - bias[row]) < 1.0e-10);
    }
    const auto derivative =
        assigned.jacobian_derivative(reference_q, reference_qd, target);
    CHECK(derivative.size() == 6 * reference_q.size());
    const auto origin_acceleration =
        assigned.forward_acceleration_kinematics(reference_q, reference_qd, zero_qdd, target);
    for (std::size_t row = 0; row < 6; ++row) {
        const double expected = row < 3
            ? origin_acceleration.angular[row]
            : origin_acceleration.linear[row - 3];
        double contracted = 0.0;
        for (std::size_t column = 0; column < reference_q.size(); ++column) {
            contracted += derivative[column * 6 + row] * reference_qd[column];
        }
        CHECK(std::abs(contracted - expected) < 1.0e-10);
    }

    const auto velocity = assigned.forward_velocity_kinematics(q, q, target);
    const auto acceleration = assigned.forward_acceleration_kinematics(q, q, q, target);
    for (double value : velocity.angular) CHECK(std::abs(value) < 1.0e-12);
    for (double value : velocity.linear) CHECK(std::abs(value) < 1.0e-12);
    for (double value : acceleration.angular) CHECK(std::abs(value) < 1.0e-12);
    for (double value : acceleration.linear) CHECK(std::abs(value) < 1.0e-12);

    DyniboLoad load{};
    load.link_id = target;
    load.force[1] = 1.0;
    CHECK(assigned.gravity(q, {load}) != gravity);

    dynibo::Robot floating(argv[1], DYNIBO_BASE_FLOATING);
    const auto floating_target = floating.link_id("test_link_4");
    const auto base_translation = read_reference(argv[2], "floating_base_translation", 3);
    const auto base_rotation = read_reference(argv[2], "floating_base_rotation_xyzw", 4);
    const auto base_velocity = read_reference(argv[2], "floating_base_velocity", 6);
    const auto base_acceleration = read_reference(argv[2], "floating_base_acceleration", 6);
    DyniboPose floating_base{};
    std::copy(base_translation.begin(), base_translation.end(), floating_base.translation);
    std::copy(base_rotation.begin(), base_rotation.end(), floating_base.rotation_xyzw);
    DyniboTwist floating_velocity{};
    DyniboTwist floating_acceleration{};
    std::copy_n(base_velocity.begin(), 3, floating_velocity.angular);
    std::copy_n(base_velocity.begin() + 3, 3, floating_velocity.linear);
    std::copy_n(base_acceleration.begin(), 3, floating_acceleration.angular);
    std::copy_n(base_acceleration.begin() + 3, 3, floating_acceleration.linear);
    floating.set_floating_base_state(
        floating_base, floating_velocity, floating_acceleration);
    const auto floating_pose = floating.forward_kinematics(reference_q, floating_target);
    const auto floating_translation =
        read_reference(argv[2], "floating_fk_translation", 3);
    for (std::size_t index = 0; index < 3; ++index) {
        CHECK(std::abs(floating_pose.translation[index] - floating_translation[index]) < 2.0e-12);
    }
    const auto floating_gravity = floating.gravity(reference_q);
    const auto expected_floating_gravity =
        read_reference(argv[2], "floating_gravity", 10);
    const auto floating_rnea =
        floating.inverse_dynamics(reference_q, reference_qd, reference_qdd);
    const auto expected_floating_rnea = read_reference(argv[2], "floating_rnea", 10);
    for (std::size_t index = 0; index < 10; ++index) {
        CHECK(std::abs(floating_gravity[index] - expected_floating_gravity[index]) < 2.0e-10);
        CHECK(std::abs(floating_rnea[index] - expected_floating_rnea[index]) < 2.0e-10);
    }
    std::vector<double> expected_floating_acceleration(base_acceleration);
    expected_floating_acceleration.insert(
        expected_floating_acceleration.end(), reference_qdd.begin(), reference_qdd.end());
    const auto floating_recovered =
        floating.forward_dynamics(reference_q, reference_qd, floating_rnea);
    for (std::size_t index = 0; index < 10; ++index) {
        CHECK(std::abs(floating_recovered[index] - expected_floating_acceleration[index]) < 2.0e-9);
    }
    const auto floating_load_values = read_reference(argv[2], "floating_load", 6);
    DyniboLoad floating_load{};
    floating_load.link_id = floating_target;
    std::copy_n(floating_load_values.begin(), 3, floating_load.torque);
    std::copy_n(floating_load_values.begin() + 3, 3, floating_load.force);
    const auto floating_loaded_rnea = floating.inverse_dynamics(
        reference_q, reference_qd, reference_qdd, {floating_load});
    const auto expected_floating_loaded_rnea =
        read_reference(argv[2], "floating_rnea_loaded", 10);
    for (std::size_t index = 0; index < 10; ++index) {
        CHECK(std::abs(floating_loaded_rnea[index] -
                       expected_floating_loaded_rnea[index]) < 2.0e-10);
    }
    const auto floating_loaded_recovered = floating.forward_dynamics(
        reference_q, reference_qd, floating_loaded_rnea, {floating_load});
    for (std::size_t index = 0; index < 10; ++index) {
        CHECK(std::abs(floating_loaded_recovered[index] -
                       expected_floating_acceleration[index]) < 2.0e-9);
    }

    bool caught = false;
    try {
        static_cast<void>(assigned.link_id("missing"));
    } catch (const dynibo::Error& error) {
        caught = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT
            && std::string(error.what()).find("does not exist") != std::string::npos;
    }
    CHECK(caught);

    const std::vector<double> short_q(q.size() - 1, 0.0);
    caught = false;
    try {
        static_cast<void>(assigned.forward_velocity_kinematics(q, short_q, target));
    } catch (const dynibo::Error& error) {
        caught = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT
            && std::string(error.what()).find("same length") != std::string::npos;
    }
    CHECK(caught);
    caught = false;
    try {
        static_cast<void>(assigned.forward_acceleration_kinematics(q, q, short_q, target));
    } catch (const dynibo::Error& error) {
        caught = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT
            && std::string(error.what()).find("same length") != std::string::npos;
    }
    CHECK(caught);
    caught = false;
    try {
        static_cast<void>(assigned.inverse_dynamics(q, short_q, q));
    } catch (const dynibo::Error& error) {
        caught = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT
            && std::string(error.what()).find("same length") != std::string::npos;
    }
    CHECK(caught);
    caught = false;
    try {
        static_cast<void>(assigned.forward_dynamics(q, short_q, q));
    } catch (const dynibo::Error& error) {
        caught = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT
            && std::string(error.what()).find("same length") != std::string::npos;
    }
    CHECK(caught);
    caught = false;
    try {
        static_cast<void>(assigned.velocity_product_forces(q, short_q));
    } catch (const dynibo::Error& error) {
        caught = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT
            && std::string(error.what()).find("same length") != std::string::npos;
    }
    CHECK(caught);
    caught = false;
    try {
        static_cast<void>(assigned.jacobian_derivative(q, short_q, target));
    } catch (const dynibo::Error& error) {
        caught = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT
            && std::string(error.what()).find("same length") != std::string::npos;
    }
    CHECK(caught);
    return 0;
}
