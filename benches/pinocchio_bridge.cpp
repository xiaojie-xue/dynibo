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
  explicit PinocchioBenchContext(pinocchio::Model model_in)
      : model(std::move(model_in)),
        data(model),
        end_joint(model.njoints - 1),
        jacobian(6, model.nv) {
    model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, 9.80665);
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
    return new PinocchioBenchContext(std::move(model));
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

}  // extern "C"

