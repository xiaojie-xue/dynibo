mod forward_kinematics;
mod inverse_kinematics;
mod jacobian;

pub use inverse_kinematics::InverseKinematicsOptions;

use crate::{Frame, Result};

use super::Model;

impl Model {
    fn prepare_ancestor_path(&self, target_index: usize, path: &mut [usize]) -> usize {
        let mut current = target_index;
        let mut depth = 0;
        while current != 0 {
            let joint_index = current - 1;
            path[depth] = joint_index;
            depth += 1;
            current = self.parent_link_indices[joint_index];
        }
        depth
    }

    fn target_frames_kernel(&self, q: &[f64], path: &[usize], frames: &mut [Frame]) -> Result<()> {
        self.validate_slice("q", q)?;
        self.validate_slice_length("frame workspace", frames.len(), self.model_joint_count())?;
        let mut frame = Frame::identity();
        for &joint_index in path.iter().rev() {
            frame *= self.joint_kinematics[joint_index].frame(self.joint_value(q, joint_index));
            frames[joint_index] = frame;
        }
        Ok(())
    }
}
