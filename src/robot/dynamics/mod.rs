mod forward_dynamics;
mod inverse_dynamics;
mod mass_matrix;

use crate::{Frame, Result, Wrench};

use super::{FLOATING_BASE_DOF, Model};

impl Model {
    #[inline]
    pub(super) fn populate_parent_from_child(
        &self,
        q: &[f64],
        parent_from_child: &mut [Frame],
    ) -> Result<()> {
        self.validate_slice_length(
            "transform workspace",
            parent_from_child.len(),
            self.model_joint_count(),
        )?;
        for (joint_index, frame) in parent_from_child.iter_mut().enumerate() {
            *frame = self.joint_kinematics[joint_index].frame(self.joint_value(q, joint_index));
        }
        Ok(())
    }
}

fn write_world_wrench(base_frame: &Frame, local: Wrench, output: &mut [f64]) {
    let torque = base_frame.rotation * local.torque;
    let force = base_frame.rotation * local.force;
    output[..3].copy_from_slice(torque.as_slice());
    output[3..6].copy_from_slice(force.as_slice());
}

fn wrench_component(wrench: Wrench, index: usize) -> f64 {
    if index < 3 {
        wrench.torque[index]
    } else {
        wrench.force[index - 3]
    }
}

fn write_wrench_to_column(output: &mut [f64], rows: usize, column: usize, wrench: Wrench) {
    for row in 0..FLOATING_BASE_DOF {
        output[column * rows + row] = wrench_component(wrench, row);
    }
}
