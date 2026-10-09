//! Shared fixtures and adapters for this integration-test target.

// Several regression inputs deliberately preserve the decimal constants used
// by the original C++ tests instead of replacing them with exact PI fractions.
#![allow(clippy::approx_constant)]

use std::{
    f64::consts::{FRAC_PI_2, PI},
    path::PathBuf,
};

use approx::{assert_abs_diff_eq, assert_relative_eq};
use dynibo::{Error, Frame, IndexedLoad, InverseKinematicsOptions, LinkId, Robot, Twist, Wrench};
use nalgebra::{Isometry3, Matrix3, SMatrix, SVector, Translation3, UnitQuaternion, Vector3};

type JointVector<const N: usize> = SVector<f64, N>;
type Jacobian<const N: usize> = SMatrix<f64, 6, N>;

const STANDARD_GRAVITY: f64 = 9.80665;

#[derive(Clone, Copy)]
struct Load {
    link: LinkId,
    wrench: Wrench,
}

trait DynamicTestApi {
    fn test_forward_kinematics<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        target: LinkId,
    ) -> dynibo::Result<Frame>;
    fn test_jacobian<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        target: LinkId,
    ) -> dynibo::Result<Jacobian<N>>;
    fn test_inverse_kinematics<const N: usize>(
        &mut self,
        initial_q: &JointVector<N>,
        target: LinkId,
        desired: &Frame,
        options: InverseKinematicsOptions,
    ) -> dynibo::Result<JointVector<N>>;
    fn test_forward_velocity_kinematics<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        target: LinkId,
        base: &Frame,
        tool: &Frame,
    ) -> dynibo::Result<Twist>;
    fn test_forward_acceleration_kinematics<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        target: LinkId,
    ) -> dynibo::Result<Twist>;
    fn test_gravity<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        base: &Frame,
        loads: &[Load],
    ) -> dynibo::Result<JointVector<N>>;
    fn test_inverse_dynamics<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        loads: &[Load],
    ) -> dynibo::Result<JointVector<N>>;
}

impl DynamicTestApi for Robot {
    fn test_forward_kinematics<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        target: LinkId,
    ) -> dynibo::Result<Frame> {
        self.forward_kinematics(q.as_slice(), target)
    }

    fn test_jacobian<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        target: LinkId,
    ) -> dynibo::Result<Jacobian<N>> {
        let mut output = Jacobian::<N>::zeros();
        self.jacobian(q.as_slice(), target, output.as_mut_slice())?;
        Ok(output)
    }

    fn test_inverse_kinematics<const N: usize>(
        &mut self,
        initial_q: &JointVector<N>,
        target: LinkId,
        desired: &Frame,
        options: InverseKinematicsOptions,
    ) -> dynibo::Result<JointVector<N>> {
        let mut output = JointVector::<N>::zeros();
        self.inverse_kinematics(
            initial_q.as_slice(),
            target,
            desired,
            options,
            output.as_mut_slice(),
        )?;
        Ok(output)
    }

    fn test_forward_velocity_kinematics<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        target: LinkId,
        base: &Frame,
        tool: &Frame,
    ) -> dynibo::Result<Twist> {
        self.set_base_frame(*base)?;
        self.forward_velocity_kinematics(q.as_slice(), qd.as_slice(), target, tool)
    }

    fn test_forward_acceleration_kinematics<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        target: LinkId,
    ) -> dynibo::Result<Twist> {
        self.forward_acceleration_kinematics(q.as_slice(), qd.as_slice(), qdd.as_slice(), target)
    }

    fn test_gravity<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        base: &Frame,
        loads: &[Load],
    ) -> dynibo::Result<JointVector<N>> {
        let loads = loads
            .iter()
            .map(|load| {
                Ok(IndexedLoad {
                    link: load.link,
                    wrench: load.wrench,
                })
            })
            .collect::<dynibo::Result<Vec<_>>>()?;
        let mut output = JointVector::<N>::zeros();
        self.set_base_frame(*base)?;
        self.gravity(q.as_slice(), &loads, output.as_mut_slice())?;
        Ok(output)
    }

    fn test_inverse_dynamics<const N: usize>(
        &mut self,
        q: &JointVector<N>,
        qd: &JointVector<N>,
        qdd: &JointVector<N>,
        loads: &[Load],
    ) -> dynibo::Result<JointVector<N>> {
        let loads = loads
            .iter()
            .map(|load| {
                Ok(IndexedLoad {
                    link: load.link,
                    wrench: load.wrench,
                })
            })
            .collect::<dynibo::Result<Vec<_>>>()?;
        let mut output = JointVector::<N>::zeros();
        self.inverse_dynamics(
            q.as_slice(),
            qd.as_slice(),
            qdd.as_slice(),
            &loads,
            output.as_mut_slice(),
        )?;
        Ok(output)
    }
}

fn test_arm() -> Robot {
    Robot::from_urdf(urdf_path("test_arm.urdf")).expect("test URDF must load")
}

fn end_link(arm: &Robot) -> LinkId {
    arm.link_id_at(arm.link_count() - 1)
        .expect("test chain must have an end link")
}

fn urdf_path(file_name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(file_name)
}

fn tree_arm() -> Robot {
    Robot::from_urdf(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/test_tree_7.urdf"))
        .expect("tree URDF must load")
}

mod dynamics;
mod inverse_kinematics;
mod kinematics;
mod model;
