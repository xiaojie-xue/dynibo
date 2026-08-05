#include <dynibo/dynibo.hpp>

#include <cmath>
#include <iostream>
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

int main(int argc, char** argv) {
    CHECK(argc == 2);
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

    const std::vector<double> reference_q{0.2, 1.0, -0.7, 0.4};
    const std::vector<double> reference_qd{-0.3, 0.5, -0.2, 0.8};
    const std::vector<double> reference_qdd{0.7, -0.4, 0.1, 0.3};
    const std::vector<double> expected_gravity{
        1.7763568394002505e-15, 39.629058959145354,
        17.60815765611755, 0.053134179784508524};
    const std::vector<double> expected_dynamics{
        1.7649236924309104, 38.319908179086525,
        17.136450444507805, 0.05169960944426318};
    const auto reference_gravity = assigned.gravity(reference_q);
    const auto reference_dynamics =
        assigned.inverse_dynamics(reference_q, reference_qd, reference_qdd);
    for (std::size_t index = 0; index < reference_q.size(); ++index) {
        CHECK(std::abs(reference_gravity[index] - expected_gravity[index]) < 2.0e-10);
        CHECK(std::abs(reference_dynamics[index] - expected_dynamics[index]) < 2.0e-10);
    }

    const auto velocity = assigned.forward_velocity(q, q, target);
    const auto acceleration = assigned.forward_acceleration(q, q, q, target);
    for (double value : velocity.angular) CHECK(std::abs(value) < 1.0e-12);
    for (double value : velocity.linear) CHECK(std::abs(value) < 1.0e-12);
    for (double value : acceleration.angular) CHECK(std::abs(value) < 1.0e-12);
    for (double value : acceleration.linear) CHECK(std::abs(value) < 1.0e-12);

    DyniboLoad load{};
    load.link_id = target;
    load.force[1] = 1.0;
    CHECK(assigned.gravity(q, dynibo::identity_pose(), {load}) != gravity);

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
        static_cast<void>(assigned.forward_velocity(q, short_q, target));
    } catch (const dynibo::Error& error) {
        caught = error.status() == DYNIBO_STATUS_INVALID_ARGUMENT
            && std::string(error.what()).find("same length") != std::string::npos;
    }
    CHECK(caught);
    caught = false;
    try {
        static_cast<void>(assigned.forward_acceleration(q, q, short_q, target));
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
    return 0;
}
