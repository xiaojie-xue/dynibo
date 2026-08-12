#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <exception>
#include <memory>
#include <vector>

namespace {

struct PinocchioBenchContext {
  explicit PinocchioBenchContext(pinocchio::Model model_in,
                                 pinocchio::JointIndex end_joint_in,
                                 pinocchio::FrameIndex target_frame_in = 0)
      : model(std::move(model_in)),
        data(model),
        end_joint(end_joint_in),
        target_frame(target_frame_in),
        jacobian(6, model.nv) {
    model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.80665);
    jacobian.setZero();
  }

  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::JointIndex end_joint;
  pinocchio::FrameIndex target_frame;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
};

using ConfigMap = Eigen::Map<const Eigen::VectorXd>;

}  // namespace

extern "C" {

void* dynibo_pinocchio_create(const char* urdf_path) noexcept {
  try {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    const pinocchio::JointIndex end_joint = model.njoints - 1;
    return new PinocchioBenchContext(std::move(model), end_joint);
  } catch (const std::exception&) {
    return nullptr;
  }
}

void* dynibo_pinocchio_create_for_joint(const char* urdf_path,
                                      const char* end_joint_name) noexcept {
  try {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    const pinocchio::JointIndex end_joint = model.getJointId(end_joint_name);
    if (end_joint == model.njoints) {
      return nullptr;
    }
    return new PinocchioBenchContext(std::move(model), end_joint);
  } catch (const std::exception&) {
    return nullptr;
  }
}

void* dynibo_pinocchio_create_for_frame(const char* urdf_path,
                                      const char* frame_name) noexcept {
  try {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    const pinocchio::FrameIndex frame = model.getFrameId(frame_name, pinocchio::BODY);
    if (frame == model.nframes) {
      return nullptr;
    }
    const pinocchio::JointIndex parent_joint = model.frames[frame].parentJoint;
    return new PinocchioBenchContext(std::move(model), parent_joint, frame);
  } catch (const std::exception&) {
    return nullptr;
  }
}

void* dynibo_pinocchio_create_floating_for_frame(
    const char* urdf_path, const char* frame_name) noexcept {
  try {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(),
                               model);
    const pinocchio::FrameIndex frame = model.getFrameId(frame_name, pinocchio::BODY);
    if (frame == model.nframes) {
      return nullptr;
    }
    const pinocchio::JointIndex parent_joint = model.frames[frame].parentJoint;
    return new PinocchioBenchContext(std::move(model), parent_joint, frame);
  } catch (const std::exception&) {
    return nullptr;
  }
}

void dynibo_pinocchio_destroy(void* raw_context) noexcept {
  delete static_cast<PinocchioBenchContext*>(raw_context);
}

std::size_t dynibo_pinocchio_dof(const void* raw_context) noexcept {
  const auto* context = static_cast<const PinocchioBenchContext*>(raw_context);
  return context->model.nv;
}

std::size_t dynibo_pinocchio_configuration_size(const void* raw_context) noexcept {
  const auto* context = static_cast<const PinocchioBenchContext*>(raw_context);
  return context->model.nq;
}

void dynibo_pinocchio_neutral_configuration(const void* raw_context,
                                          double* q) noexcept {
  const auto* context = static_cast<const PinocchioBenchContext*>(raw_context);
  Eigen::Map<Eigen::VectorXd> configuration(q, context->model.nq);
  configuration = pinocchio::neutral(context->model);
}

std::size_t dynibo_pinocchio_joint_configuration_index(
    const void* raw_context, const char* joint_name) noexcept {
  try {
    const auto* context = static_cast<const PinocchioBenchContext*>(raw_context);
    const pinocchio::JointIndex joint = context->model.getJointId(joint_name);
    if (joint == context->model.njoints || context->model.nqs[joint] == 0) {
      return context->model.nq;
    }
    return context->model.idx_qs[joint];
  } catch (const std::exception&) {
    return static_cast<const PinocchioBenchContext*>(raw_context)->model.nq;
  }
}

std::size_t dynibo_pinocchio_joint_configuration_dimension(
    const void* raw_context, const char* joint_name) noexcept {
  try {
    const auto* context = static_cast<const PinocchioBenchContext*>(raw_context);
    const pinocchio::JointIndex joint = context->model.getJointId(joint_name);
    if (joint == context->model.njoints) {
      return 0;
    }
    return context->model.nqs[joint];
  } catch (const std::exception&) {
    return 0;
  }
}

std::size_t dynibo_pinocchio_joint_velocity_index(const void* raw_context,
                                                const char* joint_name) noexcept {
  try {
    const auto* context = static_cast<const PinocchioBenchContext*>(raw_context);
    const pinocchio::JointIndex joint = context->model.getJointId(joint_name);
    if (joint == context->model.njoints || context->model.nvs[joint] != 1) {
      return context->model.nv;
    }
    return context->model.idx_vs[joint];
  } catch (const std::exception&) {
    return static_cast<const PinocchioBenchContext*>(raw_context)->model.nv;
  }
}

double dynibo_pinocchio_noop(const void*, const double* q) noexcept { return q[0]; }

double dynibo_pinocchio_forward_kinematics(void* raw_context, const double* q) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  pinocchio::forwardKinematics(context->model, context->data, configuration);
  const auto& placement = context->data.oMi[context->end_joint];
  return placement.translation().sum() + placement.rotation()(0, 0);
}

double dynibo_pinocchio_jacobian(void* raw_context, const double* q) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  pinocchio::computeJointJacobians(context->model, context->data, configuration);
  context->jacobian.setZero();
  pinocchio::getJointJacobian(context->model, context->data, context->end_joint,
                              pinocchio::LOCAL_WORLD_ALIGNED, context->jacobian);
  return context->jacobian.sum();
}

double dynibo_pinocchio_gravity(void* raw_context, const double* q) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  return pinocchio::computeGeneralizedGravity(context->model, context->data, configuration).sum();
}

double dynibo_pinocchio_rnea(void* raw_context, const double* q, const double* qd,
                           const double* qdd) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap velocity(qd, context->model.nv);
  const ConfigMap acceleration(qdd, context->model.nv);
  return pinocchio::rnea(context->model, context->data, configuration, velocity, acceleration).sum();
}

void dynibo_pinocchio_gravity_values(void* raw_context, const double* q,
                                   double* gravity) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  Eigen::Map<Eigen::VectorXd> gravity_map(gravity, context->model.nv);
  gravity_map =
      pinocchio::computeGeneralizedGravity(context->model, context->data, configuration);
}

void dynibo_pinocchio_rnea_values(void* raw_context, const double* q, const double* qd,
                                const double* qdd, double* torque) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap velocity(qd, context->model.nv);
  const ConfigMap acceleration(qdd, context->model.nv);
  Eigen::Map<Eigen::VectorXd> torque_map(torque, context->model.nv);
  torque_map =
      pinocchio::rnea(context->model, context->data, configuration, velocity, acceleration);
}

void dynibo_pinocchio_mass_matrix_values(void* raw_context, const double* q,
                                       double* mass) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  pinocchio::crba(context->model, context->data, configuration);
  context->data.M.triangularView<Eigen::Lower>() =
      context->data.M.transpose().triangularView<Eigen::Lower>();
  Eigen::Map<Eigen::MatrixXd> mass_map(mass, context->model.nv, context->model.nv);
  mass_map = context->data.M;
}

void dynibo_pinocchio_coriolis_values(void* raw_context, const double* q, const double* qd,
                                    double* coriolis) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap velocity(qd, context->model.nv);
  Eigen::Map<Eigen::MatrixXd> coriolis_map(coriolis, context->model.nv, context->model.nv);
  coriolis_map =
      pinocchio::computeCoriolisMatrix(context->model, context->data, configuration, velocity);
}

void dynibo_pinocchio_link_frame_values(void* raw_context, const double* q,
                                      double* rotation,
                                      double* translation) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  pinocchio::forwardKinematics(context->model, context->data, configuration);
  const auto& placement = pinocchio::updateFramePlacement(
      context->model, context->data, context->target_frame);
  Eigen::Map<Eigen::Matrix<double, 3, 3>> rotation_map(rotation);
  Eigen::Map<Eigen::Vector3d> translation_map(translation);
  rotation_map = placement.rotation();
  translation_map = placement.translation();
}

void dynibo_pinocchio_link_jacobian_values(void* raw_context, const double* q,
                                         double* jacobian) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  context->jacobian.setZero();
  pinocchio::computeFrameJacobian(
      context->model, context->data, configuration, context->target_frame,
      pinocchio::LOCAL_WORLD_ALIGNED, context->jacobian);
  Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian_map(
      jacobian, 6, context->model.nv);
  jacobian_map = context->jacobian;
}

void dynibo_pinocchio_link_jacobian_derivative_values(void* raw_context, const double* q,
                                                    const double* qd,
                                                    double* derivative) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap velocity(qd, context->model.nv);
  pinocchio::computeJointJacobiansTimeVariation(context->model, context->data, configuration,
                                               velocity);
  pinocchio::updateFramePlacements(context->model, context->data);
  context->jacobian.setZero();
  pinocchio::getFrameJacobianTimeVariation(context->model, context->data, context->target_frame,
                                          pinocchio::LOCAL_WORLD_ALIGNED, context->jacobian);
  Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic>> derivative_map(derivative, 6,
                                                                      context->model.nv);
  derivative_map = context->jacobian;
}

void dynibo_pinocchio_link_velocity_values(void* raw_context, const double* q,
                                         const double* qd,
                                         double* velocity) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap joint_velocity(qd, context->model.nv);
  pinocchio::forwardKinematics(context->model, context->data, configuration,
                              joint_velocity);
  const auto spatial_velocity = pinocchio::getFrameVelocity(
      context->model, context->data, context->target_frame,
      pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> velocity_map(velocity);
  velocity_map.head<3>() = spatial_velocity.angular();
  velocity_map.tail<3>() = spatial_velocity.linear();
}

void dynibo_pinocchio_link_acceleration_values(
    void* raw_context, const double* q, const double* qd, const double* qdd,
    double* acceleration) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap joint_velocity(qd, context->model.nv);
  const ConfigMap joint_acceleration(qdd, context->model.nv);
  pinocchio::forwardKinematics(context->model, context->data, configuration,
                              joint_velocity, joint_acceleration);
  const auto classical_acceleration = pinocchio::getFrameClassicalAcceleration(
      context->model, context->data, context->target_frame,
      pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> acceleration_map(acceleration);
  acceleration_map.head<3>() = classical_acceleration.angular();
  acceleration_map.tail<3>() = classical_acceleration.linear();
}

void dynibo_pinocchio_rnea_with_link_load_values(
    void* raw_context, const double* q, const double* qd, const double* qdd,
    const double* load, double* torque) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap velocity(qd, context->model.nv);
  const ConfigMap acceleration(qdd, context->model.nv);
  const Eigen::Map<const Eigen::Vector3d> load_torque(load);
  const Eigen::Map<const Eigen::Vector3d> load_force(load + 3);
  const pinocchio::Force frame_load(load_force, load_torque);
  pinocchio::container::aligned_vector<pinocchio::Force> external_forces(
      context->model.njoints, pinocchio::Force::Zero());
  const auto& frame = context->model.frames[context->target_frame];
  // Dynibo's load is a generalized resistance added to the required joint
  // effort. Pinocchio's fext is a physical force subtracted by RNEA.
  external_forces[frame.parentJoint] = frame.placement.act(-frame_load);
  Eigen::Map<Eigen::VectorXd> torque_map(torque, context->model.nv);
  torque_map = pinocchio::rnea(context->model, context->data, configuration,
                              velocity, acceleration, external_forces);
}

void dynibo_pinocchio_floating_rnea_values(
    void* raw_context, const double* q, const double* qd, const double* qdd,
    const double* base_translation, const double* base_rotation_xyzw,
    const double* base_velocity, const double* base_acceleration,
    double* torque) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  Eigen::VectorXd configuration = ConfigMap(q, context->model.nq);
  Eigen::VectorXd velocity = ConfigMap(qd, context->model.nv);
  Eigen::VectorXd acceleration = ConfigMap(qdd, context->model.nv);
  configuration.head<3>() = Eigen::Map<const Eigen::Vector3d>(base_translation);
  configuration.segment<4>(3) =
      Eigen::Map<const Eigen::Matrix<double, 4, 1>>(base_rotation_xyzw);
  const Eigen::Quaterniond orientation(
      base_rotation_xyzw[3], base_rotation_xyzw[0], base_rotation_xyzw[1],
      base_rotation_xyzw[2]);
  const Eigen::Matrix3d world_to_base = orientation.toRotationMatrix().transpose();
  const Eigen::Vector3d base_linear_velocity =
      world_to_base * Eigen::Map<const Eigen::Vector3d>(base_velocity + 3);
  const Eigen::Vector3d base_angular_velocity =
      world_to_base * Eigen::Map<const Eigen::Vector3d>(base_velocity);
  velocity.head<3>() = base_linear_velocity;
  velocity.segment<3>(3) = base_angular_velocity;
  // Pinocchio accepts spatial acceleration, while Dynibo's public base input is
  // classical acceleration at the base origin.
  acceleration.head<3>() =
      world_to_base * Eigen::Map<const Eigen::Vector3d>(base_acceleration + 3)
      - base_angular_velocity.cross(base_linear_velocity);
  acceleration.segment<3>(3) =
      world_to_base * Eigen::Map<const Eigen::Vector3d>(base_acceleration);
  Eigen::Map<Eigen::VectorXd> torque_map(torque, context->model.nv);
  torque_map = pinocchio::rnea(context->model, context->data, configuration,
                              velocity, acceleration);
}

}  // extern "C"
