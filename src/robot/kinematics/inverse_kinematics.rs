use nalgebra::{SMatrix, SVector};

use crate::{BaseMode, BaseState, Error, Frame, Result};

use super::super::{LinkId, Robot, Workspace};

/// Configuration for damped-least-squares inverse kinematics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverseKinematicsOptions {
    /// Maximum number of joint updates.
    pub max_iterations: usize,
    /// Maximum accepted Euclidean position error, in metres.
    pub translation_tolerance: f64,
    /// Maximum accepted rotation-vector norm, in radians.
    pub rotation_tolerance: f64,
    /// Damping factor `lambda` in `J^T (J J^T + lambda^2 I)^-1`.
    pub damping: f64,
    /// Maximum Euclidean norm of one joint update.
    pub max_step_norm: f64,
}

impl Default for InverseKinematicsOptions {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            translation_tolerance: 1.0e-6,
            rotation_tolerance: 1.0e-6,
            damping: 1.0e-3,
            max_step_norm: 0.5,
        }
    }
}

struct IkScratch<'a> {
    frames: &'a mut [Frame],
    jacobian: &'a mut [f64],
    q_work: &'a mut [f64],
    step: &'a mut [f64],
    ancestor_path: &'a mut [usize],
}

impl Robot {
    /// Writes a runtime-sized inverse-kinematics solution using the supplied options.
    ///
    /// Each iteration applies a damped-least-squares update,
    ///
    /// $$
    /// \Delta q = J^T\left(JJ^T + \lambda^2 I\right)^{-1} e,
    /// \qquad q_{k+1} = q_k + \Delta q.
    /// $$
    ///
    /// where `e` combines target translation and rotation-vector errors.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid lengths, link ID, workspace, solver input,
    /// numerical failure, limits, or non-convergence.
    #[allow(clippy::too_many_arguments)]
    pub fn inverse_kinematics(
        &self,
        base: &BaseState,
        initial_q: &[f64],
        target: LinkId,
        desired: &Frame,
        options: InverseKinematicsOptions,
        workspace: &mut Workspace,
        output: &mut [f64],
    ) -> Result<()> {
        self.validate_base_state(base)?;
        self.validate_workspace(workspace)?;
        if self.base_mode() == BaseMode::Floating {
            return Err(Error::FloatingBaseIkUnsupported);
        }
        self.validate_slice("initial_q", initial_q)?;
        self.validate_joint_output("inverse kinematics output", output)?;
        let target_index = self.validate_link_id(target)?;
        let local_desired = base.frame().inverse() * *desired;
        self.inverse_kinematics_kernel(
            initial_q,
            target_index,
            &local_desired,
            options,
            IkScratch {
                frames: &mut workspace.frames,
                jacobian: &mut workspace.jacobian,
                q_work: &mut workspace.q_work,
                step: &mut workspace.step,
                ancestor_path: &mut workspace.ancestor_path,
            },
        )?;
        output.copy_from_slice(&workspace.q_work);
        Ok(())
    }

    fn inverse_kinematics_kernel(
        &self,
        initial_q: &[f64],
        target_index: usize,
        desired: &Frame,
        options: InverseKinematicsOptions,
        scratch: IkScratch<'_>,
    ) -> Result<()> {
        let IkScratch {
            frames,
            jacobian,
            q_work,
            step,
            ancestor_path,
        } = scratch;
        self.validate_slice("initial_q", initial_q)?;
        self.validate_joint_output("IK joint workspace", q_work)?;
        self.validate_joint_output("IK step workspace", step)?;
        validate_inverse_kinematics_options(options)?;
        if !initial_q.iter().all(|value| value.is_finite()) {
            return Err(Error::NonFiniteIkInput {
                input: "initial joint vector",
            });
        }
        if !desired
            .translation
            .vector
            .iter()
            .chain(desired.rotation.coords.iter())
            .all(|value| value.is_finite())
        {
            return Err(Error::NonFiniteIkInput {
                input: "target frame",
            });
        }
        q_work.copy_from_slice(initial_q);
        let depth = self.prepare_ancestor_path(target_index, ancestor_path);
        let path = &ancestor_path[..depth];
        jacobian.fill(0.0);
        let damping_squared = options.damping * options.damping;
        for iteration in 0..=options.max_iterations {
            self.target_frames_kernel(q_work, path, frames)?;
            let current = self.jacobian_kernel(frames, target_index, path, jacobian, false)?;
            let translation_error = desired.translation.vector - current.translation.vector;
            let rotation_error = (desired.rotation * current.rotation.inverse()).scaled_axis();
            let translation_error_norm = translation_error.norm();
            let rotation_error_norm = rotation_error.norm();
            if translation_error_norm <= options.translation_tolerance
                && rotation_error_norm <= options.rotation_tolerance
            {
                self.validate_inverse_kinematics_solution(q_work)?;
                return Ok(());
            }
            if iteration == options.max_iterations {
                return Err(Error::IkNotConverged {
                    iterations: options.max_iterations,
                    translation_error: translation_error_norm,
                    rotation_error: rotation_error_norm,
                });
            }
            let error = SVector::<f64, 6>::from_iterator(
                rotation_error
                    .iter()
                    .chain(translation_error.iter())
                    .copied(),
            );
            let mut regularized = SMatrix::<f64, 6, 6>::identity() * damping_squared;
            for &joint_index in path.iter().rev() {
                let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                    continue;
                };
                let column = &jacobian[6 * dof_index..6 * dof_index + 6];
                for row in 0..6 {
                    for col in 0..=row {
                        regularized[(row, col)] += column[row] * column[col];
                    }
                }
            }
            // nalgebra's Cholesky decomposition reads only the lower triangle.
            let Some(weighted_error) = regularized.cholesky().map(|factor| factor.solve(&error))
            else {
                return Err(Error::IkNumericalFailure {
                    iteration: iteration + 1,
                });
            };
            let mut step_norm_squared = 0.0;
            for &joint_index in path.iter().rev() {
                let Some(dof_index) = self.joint_dof_indices[joint_index] else {
                    continue;
                };
                let column = &jacobian[6 * dof_index..6 * dof_index + 6];
                step[dof_index] = column
                    .iter()
                    .zip(weighted_error.iter())
                    .map(|(lhs, rhs)| lhs * rhs)
                    .sum();
                step_norm_squared += step[dof_index] * step[dof_index];
            }
            let step_norm = step_norm_squared.sqrt();
            if !step_norm.is_finite() {
                return Err(Error::IkNumericalFailure {
                    iteration: iteration + 1,
                });
            }
            let scale = if step_norm > options.max_step_norm {
                options.max_step_norm / step_norm
            } else {
                1.0
            };
            for &joint_index in path.iter().rev() {
                if let Some(dof_index) = self.joint_dof_indices[joint_index] {
                    q_work[dof_index] += scale * step[dof_index];
                }
            }
        }
        unreachable!("inverse-kinematics loop always returns")
    }

    fn validate_inverse_kinematics_solution(&self, q: &[f64]) -> Result<()> {
        for (&joint_index, &position) in self.active_joint_indices.iter().zip(q) {
            let joint = &self.joints[joint_index];
            if joint.is_over_limit(position) {
                return Err(Error::IkJointLimitViolation {
                    joint_index,
                    joint: joint.name().to_owned(),
                    position,
                    lower: joint.lower_limit(),
                    upper: joint.upper_limit(),
                });
            }
        }
        Ok(())
    }
}

fn validate_inverse_kinematics_options(options: InverseKinematicsOptions) -> Result<()> {
    if options.max_iterations == 0 {
        return Err(Error::InvalidIkOptions {
            option: "max_iterations",
            reason: "must be greater than zero",
        });
    }
    for (name, value) in [
        ("translation_tolerance", options.translation_tolerance),
        ("rotation_tolerance", options.rotation_tolerance),
        ("damping", options.damping),
        ("max_step_norm", options.max_step_norm),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(Error::InvalidIkOptions {
                option: name,
                reason: "must be finite and greater than zero",
            });
        }
    }
    Ok(())
}
