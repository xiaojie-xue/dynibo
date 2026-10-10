mod forward_kinematics;
mod inverse_kinematics;
mod jacobian;

pub use inverse_kinematics::InverseKinematicsOptions;

use crate::{Frame, Result};

use super::Model;
use super::topology::incoming_joint_index;

impl Model {
    fn prepare_ancestor_path(&self, target_index: usize, path: &mut [usize]) -> usize {
        let mut current_link_index = target_index;
        let mut depth = 0;
        while current_link_index != 0 {
            let joint_index = incoming_joint_index(current_link_index);
            path[depth] = joint_index;
            depth += 1;
            current_link_index = self.parent_link_indices[joint_index];
        }
        depth
    }

    fn target_frames_kernel(
        &self,
        q: &[f64],
        path: &[usize],
        root_from_link: &mut [Frame],
    ) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice_length(
            "frame workspace",
            root_from_link.len(),
            self.model_joint_count(),
        )?;
        let mut frame = Frame::identity();
        for &joint_index in path.iter().rev() {
            frame *= self.joint_kinematics[joint_index].frame(self.joint_value(q, joint_index));
            root_from_link[joint_index] = frame;
        }
        Ok(())
    }
}
