use crate::{Error, Frame, JointVector, Motion, Result, RobotArm, Wrench};

/// Mapping between actuator coordinates and a model containing passive joints.
pub trait PassiveJointMap<const ACTIVE: usize, const ALL: usize> {
    fn expand(&self, active: &JointVector<ACTIVE>) -> JointVector<ALL>;
    fn reduce_force(&self, all: &JointVector<ALL>) -> JointVector<ACTIVE>;
}

/// Adapter for a robot model containing passive joints.
#[derive(Clone, Debug)]
pub struct RobotWithPassiveJoints<
    const ACTIVE: usize,
    const ALL: usize,
    M: PassiveJointMap<ACTIVE, ALL>,
> {
    arm: RobotArm,
    mapping: M,
}

impl<const ACTIVE: usize, const ALL: usize, M: PassiveJointMap<ACTIVE, ALL>>
    RobotWithPassiveJoints<ACTIVE, ALL, M>
{
    pub fn new(arm: RobotArm, mapping: M) -> Result<Self> {
        if arm.joint_count() != ALL {
            return Err(Error::WrongJointCount {
                expected: ALL,
                actual: arm.joint_count(),
            });
        }
        Ok(Self { arm, mapping })
    }

    pub const fn arm(&self) -> &RobotArm {
        &self.arm
    }

    pub fn forward_kinematics(&self, q: &JointVector<ACTIVE>) -> Result<Frame> {
        self.arm.forward_kinematics(&self.mapping.expand(q))
    }

    pub fn forward_velocity_kinematics(
        &self,
        q: &JointVector<ACTIVE>,
        qd: &JointVector<ACTIVE>,
        base: &Frame,
        tool: &Frame,
    ) -> Result<Motion> {
        self.arm.forward_velocity_kinematics(
            &self.mapping.expand(q),
            &self.mapping.expand(qd),
            base,
            tool,
        )
    }

    pub fn forward_acceleration_kinematics(
        &self,
        q: &JointVector<ACTIVE>,
        qd: &JointVector<ACTIVE>,
        qdd: &JointVector<ACTIVE>,
    ) -> Result<Motion> {
        self.arm.forward_acceleration_kinematics(
            &self.mapping.expand(q),
            &self.mapping.expand(qd),
            &self.mapping.expand(qdd),
        )
    }

    pub fn gravity_torque(
        &self,
        q: &JointVector<ACTIVE>,
        base: &Frame,
        end_load: Wrench,
    ) -> Result<(JointVector<ACTIVE>, Wrench)> {
        let (force, base_load) =
            self.arm
                .gravity_torque(&self.mapping.expand(q), base, end_load)?;
        Ok((self.mapping.reduce_force(&force), base_load))
    }

    #[allow(clippy::too_many_arguments)]
    pub fn inverse_dynamics(
        &self,
        q: &JointVector<ACTIVE>,
        qd: &JointVector<ACTIVE>,
        qdd: &JointVector<ACTIVE>,
        base_frame: &Frame,
        base_velocity: Motion,
        base_acceleration: Motion,
        end_load: Wrench,
    ) -> Result<(JointVector<ACTIVE>, Wrench)> {
        let (force, base_load) = self.arm.inverse_dynamics(
            &self.mapping.expand(q),
            &self.mapping.expand(qd),
            &self.mapping.expand(qdd),
            base_frame,
            base_velocity,
            base_acceleration,
            end_load,
        )?;
        Ok((self.mapping.reduce_force(&force), base_load))
    }
}
