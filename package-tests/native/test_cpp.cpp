#include <dyno/dyno.hpp>

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
    dyno::Robot robot(argv[1]);
    CHECK(robot.name() == "test_arm");
    CHECK(robot.joint_count() == 4);
    CHECK(robot.link_count() == 5);

    const auto target = robot.link_id("test_link_4");
    const std::vector<double> q(robot.joint_count(), 0.0);
    const auto pose = robot.forward_kinematics(q, target);
    CHECK(std::abs(pose.translation[0] - 0.62) < 1.0e-12);
    CHECK(robot.jacobian(q, target).size() == 6 * robot.joint_count());
    CHECK(robot.gravity(q).size() == robot.joint_count());
    CHECK(robot.inverse_dynamics(q, q, q).size() == robot.joint_count());
    CHECK(robot.inverse_kinematics(q, target, pose) == q);

    bool caught = false;
    try {
        static_cast<void>(robot.link_id("missing"));
    } catch (const dyno::Error& error) {
        caught = std::string(error.what()).find("does not exist") != std::string::npos;
    }
    CHECK(caught);
    return 0;
}
