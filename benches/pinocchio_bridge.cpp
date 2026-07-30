#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <exception>
#include <memory>

namespace {

struct PinocchioBenchContext {
  explicit PinocchioBenchContext(pinocchio::Model model_in,
                                 pinocchio::JointIndex end_joint_in)
      : model(std::move(model_in)),
        data(model),
        end_joint(end_joint_in),
        jacobian(6, model.nv) {
    model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.80665);
    jacobian.setZero();
  }

  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::JointIndex end_joint;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
};

using ConfigMap = Eigen::Map<const Eigen::VectorXd>;

}  // namespace

extern "C" {

void* dyno_pinocchio_create(const char* urdf_path) noexcept {
  try {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    const pinocchio::JointIndex end_joint = model.njoints - 1;
    return new PinocchioBenchContext(std::move(model), end_joint);
  } catch (const std::exception&) {
    return nullptr;
  }
}

void* dyno_pinocchio_create_for_joint(const char* urdf_path,
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

void dyno_pinocchio_destroy(void* raw_context) noexcept {
  delete static_cast<PinocchioBenchContext*>(raw_context);
}

std::size_t dyno_pinocchio_dof(const void* raw_context) noexcept {
  const auto* context = static_cast<const PinocchioBenchContext*>(raw_context);
  return context->model.nv;
}

std::size_t dyno_pinocchio_joint_velocity_index(const void* raw_context,
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

double dyno_pinocchio_noop(const void*, const double* q) noexcept { return q[0]; }

double dyno_pinocchio_forward_kinematics(void* raw_context, const double* q) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  pinocchio::forwardKinematics(context->model, context->data, configuration);
  const auto& placement = context->data.oMi[context->end_joint];
  return placement.translation().sum() + placement.rotation()(0, 0);
}

double dyno_pinocchio_jacobian(void* raw_context, const double* q) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  pinocchio::computeJointJacobians(context->model, context->data, configuration);
  context->jacobian.setZero();
  pinocchio::getJointJacobian(context->model, context->data, context->end_joint,
                              pinocchio::LOCAL_WORLD_ALIGNED, context->jacobian);
  return context->jacobian.sum();
}

double dyno_pinocchio_gravity(void* raw_context, const double* q) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  return pinocchio::computeGeneralizedGravity(context->model, context->data, configuration).sum();
}

double dyno_pinocchio_rnea(void* raw_context, const double* q, const double* qd,
                           const double* qdd) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap velocity(qd, context->model.nv);
  const ConfigMap acceleration(qdd, context->model.nv);
  return pinocchio::rnea(context->model, context->data, configuration, velocity, acceleration).sum();
}

void dyno_pinocchio_frame_values(void* raw_context, const double* q,
                                 double* rotation, double* translation) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  pinocchio::forwardKinematics(context->model, context->data, configuration);
  const auto& placement = context->data.oMi[context->end_joint];
  Eigen::Map<Eigen::Matrix<double, 3, 3>> rotation_map(rotation);
  Eigen::Map<Eigen::Vector3d> translation_map(translation);
  rotation_map = placement.rotation();
  translation_map = placement.translation();
}

void dyno_pinocchio_jacobian_values(void* raw_context, const double* q,
                                    double* jacobian) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  pinocchio::computeJointJacobians(context->model, context->data, configuration);
  context->jacobian.setZero();
  pinocchio::getJointJacobian(context->model, context->data, context->end_joint,
                              pinocchio::LOCAL_WORLD_ALIGNED, context->jacobian);
  Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian_map(
      jacobian, 6, context->model.nv);
  jacobian_map = context->jacobian;
}

void dyno_pinocchio_gravity_values(void* raw_context, const double* q,
                                   double* gravity) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  Eigen::Map<Eigen::VectorXd> gravity_map(gravity, context->model.nv);
  gravity_map =
      pinocchio::computeGeneralizedGravity(context->model, context->data, configuration);
}

void dyno_pinocchio_rnea_values(void* raw_context, const double* q, const double* qd,
                                const double* qdd, double* torque) noexcept {
  auto* context = static_cast<PinocchioBenchContext*>(raw_context);
  const ConfigMap configuration(q, context->model.nq);
  const ConfigMap velocity(qd, context->model.nv);
  const ConfigMap acceleration(qdd, context->model.nv);
  Eigen::Map<Eigen::VectorXd> torque_map(torque, context->model.nv);
  torque_map =
      pinocchio::rnea(context->model, context->data, configuration, velocity, acceleration);
}

}  // extern "C"
