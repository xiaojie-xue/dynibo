#include <dyno/dyno.hpp>

#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: " << argv[0] << " ROBOT.urdf TOOL_LINK\n";
        return 2;
    }
    try {
        dyno::Robot robot(argv[1]);
        const auto target = robot.link_id(argv[2]);
        const auto pose = robot.forward_kinematics(
            std::vector<double>(robot.joint_count(), 0.0), target);
        std::cout << robot.name() << ": translation = ["
                  << pose.translation[0] << ", " << pose.translation[1]
                  << ", " << pose.translation[2] << "]\n";
    } catch (const dyno::Error& error) {
        std::cerr << "dyno: " << error.what() << '\n';
        return 1;
    }
}
