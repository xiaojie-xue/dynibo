mod forward_dynamics;
mod inverse_dynamics;
mod mass_matrix;

use crate::{Frame, Wrench};

use super::FLOATING_BASE_DOF;

fn wrench_to_parent(transform: &Frame, wrench: Wrench) -> Wrench {
    let force = transform.rotation * wrench.force;
    Wrench::new(
        transform.rotation * wrench.torque + transform.translation.vector.cross(&force),
        force,
    )
}

fn add_wrench(lhs: Wrench, rhs: Wrench) -> Wrench {
    Wrench::new(lhs.torque + rhs.torque, lhs.force + rhs.force)
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
