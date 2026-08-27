#include <dynibo/dynibo.hpp>

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

void print_vector(const char *label, const double *values,
                  const std::size_t count) {
  std::cout << label << ": [";
  for (std::size_t index = 0; index < count; ++index) {
    std::cout << (index == 0 ? "" : ", ") << values[index];
  }
  std::cout << "]\n";
}

void print_vector(const char *label, const std::vector<double> &values) {
  print_vector(label, values.data(), values.size());
}

void print_matrix(const char *label, const std::vector<double> &values,
                  const std::size_t rows, const std::size_t columns) {
  std::cout << label << " (" << rows << " x " << columns << "):\n";
  for (std::size_t row = 0; row < rows; ++row) {
    std::cout << "  ";
    for (std::size_t column = 0; column < columns; ++column) {
      // All dynibo matrix outputs are column-major.
      std::cout << values[row + column * rows]
                << (column + 1 == columns ? "" : " ");
    }
    std::cout << '\n';
  }
}

void print_twist(const char *label, const DyniboTwist &value) {
  const double values[6] = {
      value.angular[0], value.angular[1], value.angular[2],
      value.linear[0],  value.linear[1],  value.linear[2],
  };
  print_vector(label, values, 6);
}

int run(const int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " ROBOT.urdf TOOL_LINK\n";
    return 2;
  }

  dynibo::Robot robot(argv[1]);
  const std::size_t flange = robot.link_id(argv[2]);

  // Only non-fixed joints occupy state-vector entries; fer_joint8 is fixed.
  const std::vector<double> q = {0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.7};
  const std::vector<double> qd = {0.10, -0.20, 0.15, 0.05, -0.10, 0.20, -0.05};
  const std::vector<double> qdd = {0.20, 0.10, -0.10, 0.05, 0.10, -0.05, 0.15};
  const std::vector<double> ik_initial_q = {0.05, -0.2, -0.05, -1.6,
                                            0.05, 1.4,  0.6};

  if (robot.joint_count() != q.size()) {
    std::cerr << "this Franka example expects " << q.size()
              << " non-fixed joints, got " << robot.joint_count() << '\n';
    return 1;
  }

  // forward_kinematics -- target-link pose in the world frame.
  const DyniboPose pose = robot.forward_kinematics(q, flange);

  // jacobian and jacobian_derivative -- flat column-major matrices.
  const std::vector<double> jacobian = robot.jacobian(q, flange);
  const std::vector<double> jacobian_derivative =
      robot.jacobian_derivative(q, qd, flange);

  const DyniboTwist velocity = robot.forward_velocity_kinematics(q, qd, flange);
  const DyniboTwist acceleration =
      robot.forward_acceleration_kinematics(q, qd, qdd, flange);

  // inverse_kinematics -- recover a known, reachable pose.
  const std::vector<double> ik_solution =
      robot.inverse_kinematics(ik_initial_q, flange, pose);

  // Joint-space dynamics.
  const std::vector<double> mass_matrix = robot.mass_matrix(q);
  const std::vector<double> velocity_forces =
      robot.velocity_product_forces(q, qd);

  // A link-local downward force applied at the flange origin.
  DyniboLoad load{};
  load.link_id = flange;
  load.force[2] = -5.0;
  const std::vector<DyniboLoad> loads = {load};
  const std::vector<double> gravity = robot.gravity(q, loads);
  const std::vector<double> joint_forces =
      robot.inverse_dynamics(q, qd, qdd, loads);

  std::cout << std::fixed << std::setprecision(5);
  std::cout << "loaded " << robot.name() << ": " << robot.link_count()
            << " links, " << robot.joint_count() << " non-fixed joints\n";
  print_vector("forward_kinematics translation [m]", pose.translation, 3);
  print_vector("forward_kinematics quaternion (x, y, z, w)", pose.rotation_xyzw,
               4);
  print_matrix("jacobian", jacobian, 6, robot.generalized_count());
  print_matrix("jacobian_derivative", jacobian_derivative, 6,
               robot.generalized_count());
  print_twist("forward_velocity_kinematics", velocity);
  print_twist("forward_acceleration_kinematics", acceleration);
  print_vector("inverse_kinematics", ik_solution);
  print_matrix("mass_matrix", mass_matrix, robot.generalized_count(),
               robot.generalized_count());
  print_vector("velocity_product_forces", velocity_forces);
  print_vector("gravity (including external load)", gravity);
  print_vector("inverse_dynamics (including external load)", joint_forces);
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (const dynibo::Error &error) {
    std::cerr << "dynibo: " << error.what() << '\n';
    return 1;
  }
}
