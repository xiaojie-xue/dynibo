#include <dynibo/dynibo.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

int main(int argc, char **argv) {
  try {
    const char *path = argc > 1 ? argv[1]
        : "examples/data/unitree-g1/g1_29dof_mode_11.urdf";
    dynibo::FloatingRobot robot(path);
    const auto target = robot.link_id("left_rubber_hand");
    const auto n = robot.joint_count(), g = robot.generalized_count();
    if (n != 29 || g != 35) throw std::runtime_error("expected floating G1 (29/35)");
    auto frame = dynibo::identity_pose();
    frame.translation[2] = 0.8;
    const dynibo::BaseState base(frame,
        {{0.1, -0.05, 0.08}, {0.2, 0.0, -0.1}},
        {{0.02, 0.03, -0.01}, {0.1, -0.2, 0.05}});
    // Joint-only inputs use Dynibo's breadth-first URDF joint order.
    std::vector<double> q(n, 0.0), qd(n), qdd(n);
    for (std::size_t i = 0; i < n; ++i) {
      qd[i] = 0.1 * std::cos(static_cast<double>(i + 1));
      qdd[i] = 0.2 * std::sin(static_cast<double>(i + 1));
    }
    const auto pose = robot.forward_kinematics(base, q, target);
    const auto jacobian = robot.jacobian(base, q, target);
    auto forces = robot.inverse_dynamics(base, q, qd, qdd);
    auto acceleration = robot.forward_dynamics(base, q, qd, forces);
    double error = 0.0;
    for (std::size_t i = 0; i < g; ++i) {
      const double expected = i < 3 ? base.acceleration.angular[i]
          : i < 6 ? base.acceleration.linear[i - 3] : qdd[i - 6];
      error = std::max(error, std::abs(acceleration[i] - expected));
      if (!std::isfinite(acceleration[i])) throw std::runtime_error("nonfinite ABA result");
    }
    if (error >= 1e-8) throw std::runtime_error("RNEA/ABA round-trip failed");
    std::cout << robot.name() << ": " << n << " joints, " << g
              << " generalized velocities (floating base)\n";
    std::cout << "left hand position [m]: " << pose.translation[0] << ", "
              << pose.translation[1] << ", " << pose.translation[2] << '\n';
    std::cout << "Jacobian: 6 x " << g << " (" << jacobian.size()
              << " values), column-major, angular rows first\n";
    std::cout << "RNEA base wrench [torque, force]:";
    for (std::size_t i = 0; i < 6; ++i) std::cout << ' ' << forces[i];
    std::cout << "\nRNEA joint torques:";
    for (std::size_t i = 6; i < g; ++i) std::cout << ' ' << forces[i];
    std::cout << "\nRNEA -> ABA maximum acceleration error: " << error << '\n';
    // A freely moving base has zero applied wrench; this does not model contact.
    std::fill(forces.begin(), forces.begin() + 6, 0.0);
    acceleration = robot.forward_dynamics(base, q, qd, forces);
    std::cout << "ABA unactuated-base acceleration [angular, linear]:";
    for (std::size_t i = 0; i < 6; ++i) std::cout << ' ' << acceleration[i];
    std::cout << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "g1: " << error.what() << '\n';
    return 1;
  }
}
