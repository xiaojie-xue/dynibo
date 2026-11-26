use super::*;

impl PinocchioContext {
    pub fn frame(&mut self, configuration: &[f64]) -> (Matrix3<f64>, Vector3<f64>) {
        let mut rotation = [0.0; 9];
        let mut translation = [0.0; 3];
        // SAFETY: all buffers have the dimensions required by this context.
        unsafe {
            dynibo_pinocchio_link_frame_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                rotation.as_mut_ptr(),
                translation.as_mut_ptr(),
            )
        };
        (
            Matrix3::from_column_slice(&rotation),
            Vector3::from_column_slice(&translation),
        )
    }

    pub fn jacobian(&mut self, configuration: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; 6 * self.velocity_size];
        // SAFETY: the output has `6 * model.nv` elements.
        unsafe {
            dynibo_pinocchio_link_jacobian_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_spatial_matrix_order(&pinocchio)
    }

    pub fn floating_jacobian(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
        let mut pinocchio = vec![0.0; 6 * self.velocity_size];
        unsafe {
            dynibo_pinocchio_link_jacobian_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.transform_floating_spatial_matrix(&pinocchio, None, base, Vector3::zeros())
    }

    pub fn jacobian_derivative(&mut self, configuration: &[f64], velocity: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; 6 * self.velocity_size];
        // SAFETY: the output has `6 * model.nv` elements.
        unsafe {
            dynibo_pinocchio_link_jacobian_derivative_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_spatial_matrix_order(&pinocchio)
    }

    pub fn floating_jacobian_derivative(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        base: &Frame,
        base_angular_velocity: Vector3<f64>,
    ) -> Vec<f64> {
        let mut jacobian = vec![0.0; 6 * self.velocity_size];
        let mut derivative = vec![0.0; 6 * self.velocity_size];
        unsafe {
            dynibo_pinocchio_link_jacobian_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                jacobian.as_mut_ptr(),
            );
            dynibo_pinocchio_link_jacobian_derivative_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                derivative.as_mut_ptr(),
            );
        }
        self.transform_floating_spatial_matrix(
            &derivative,
            Some(&jacobian),
            base,
            base_angular_velocity,
        )
    }

    pub fn velocity(&mut self, configuration: &[f64], velocity: &[f64]) -> [f64; 6] {
        let mut output = [0.0; 6];
        // SAFETY: input and output sizes match the context dimensions.
        unsafe {
            dynibo_pinocchio_link_velocity_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                output.as_mut_ptr(),
            )
        };
        output
    }

    pub fn acceleration(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
    ) -> [f64; 6] {
        let mut output = [0.0; 6];
        // SAFETY: input and output sizes match the context dimensions.
        unsafe {
            dynibo_pinocchio_link_acceleration_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                output.as_mut_ptr(),
            )
        };
        output
    }
}
