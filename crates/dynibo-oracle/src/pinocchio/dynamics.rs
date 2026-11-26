use super::*;

impl PinocchioContext {
    fn rnea_with_loads_raw(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let (frame_indices, load_values) = Self::load_buffers(loads);
        let mut output = vec![0.0; self.velocity_size];
        // SAFETY: state and output buffers match the model dimensions; each
        // frame index has one six-element wrench in `load_values`.
        unsafe {
            dynibo_pinocchio_rnea_with_loads_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                frame_indices.as_ptr(),
                load_values.as_ptr(),
                loads.len(),
                output.as_mut_ptr(),
            )
        };
        output
    }

    fn aba_with_loads_raw(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        generalized_forces: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        assert_eq!(generalized_forces.len(), self.velocity_size);
        let (frame_indices, load_values) = Self::load_buffers(loads);
        let mut output = vec![0.0; self.velocity_size];
        // SAFETY: state, force, and output buffers match the model dimensions;
        // each frame index has one six-element wrench in `load_values`.
        unsafe {
            dynibo_pinocchio_aba_with_loads_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                generalized_forces.as_ptr(),
                frame_indices.as_ptr(),
                load_values.as_ptr(),
                loads.len(),
                output.as_mut_ptr(),
            )
        };
        output
    }

    pub fn gravity(&mut self, configuration: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: the output has one value per Pinocchio velocity coordinate.
        unsafe {
            dynibo_pinocchio_gravity_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio)
    }

    pub fn rnea(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
    ) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions.
        unsafe {
            dynibo_pinocchio_rnea_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio)
    }

    pub fn rnea_with_loads(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let pinocchio = self.rnea_with_loads_raw(configuration, velocity, acceleration, loads);
        self.dynibo_joint_order(&pinocchio)
    }

    pub fn gravity_with_loads(
        &mut self,
        configuration: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let zero = vec![0.0; self.velocity_size];
        self.rnea_with_loads(configuration, &zero, &zero, loads)
    }

    pub fn aba(&mut self, configuration: &[f64], velocity: &[f64], torque: &[f64]) -> Vec<f64> {
        assert_eq!(torque.len(), self.joint_mappings.len());
        let mut pinocchio_torque = vec![0.0; self.velocity_size];
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                pinocchio_torque[index] = torque[joint];
            }
        }
        let mut pinocchio_acceleration = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions.
        unsafe {
            dynibo_pinocchio_aba_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio_torque.as_ptr(),
                pinocchio_acceleration.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio_acceleration)
    }

    pub fn aba_with_loads(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        torque: &[f64],
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let pinocchio_torque = self.pinocchio_joint_forces(torque);
        let acceleration =
            self.aba_with_loads_raw(configuration, velocity, &pinocchio_torque, loads);
        self.dynibo_joint_order(&acceleration)
    }

    pub fn aba_with_link_load(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        torque: &[f64],
        load: Wrench,
    ) -> Vec<f64> {
        assert_eq!(torque.len(), self.joint_mappings.len());
        let mut pinocchio_torque = vec![0.0; self.velocity_size];
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                pinocchio_torque[index] = torque[joint];
            }
        }
        let load = [
            load.torque.x,
            load.torque.y,
            load.torque.z,
            load.force.x,
            load.force.y,
            load.force.z,
        ];
        let mut pinocchio_acceleration = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions.
        unsafe {
            dynibo_pinocchio_aba_with_link_load_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio_torque.as_ptr(),
                load.as_ptr(),
                pinocchio_acceleration.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio_acceleration)
    }

    pub fn floating_aba(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        generalized_forces: &[f64],
        base: &Frame,
        base_velocity: Twist,
    ) -> Vec<f64> {
        assert_eq!(generalized_forces.len(), 6 + self.joint_mappings.len());
        let mut pinocchio_force = vec![0.0; self.velocity_size];
        let world_to_base = base.rotation.inverse();
        let local_torque = world_to_base * Vector3::from_column_slice(&generalized_forces[..3]);
        let local_force = world_to_base * Vector3::from_column_slice(&generalized_forces[3..6]);
        pinocchio_force[..3].copy_from_slice(local_force.as_slice());
        pinocchio_force[3..6].copy_from_slice(local_torque.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                pinocchio_force[index] = generalized_forces[6 + joint];
            }
        }
        let mut pinocchio_acceleration = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the free-flyer context dimensions.
        unsafe {
            dynibo_pinocchio_aba_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio_force.as_ptr(),
                pinocchio_acceleration.as_mut_ptr(),
            )
        };

        self.floating_acceleration_order(&pinocchio_acceleration, base, base_velocity)
    }

    pub fn floating_aba_with_loads(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        generalized_forces: &[f64],
        base: &Frame,
        base_velocity: Twist,
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        assert_eq!(generalized_forces.len(), 6 + self.joint_mappings.len());
        let mut pinocchio_force = vec![0.0; self.velocity_size];
        let world_to_base = base.rotation.inverse();
        let local_torque = world_to_base * Vector3::from_column_slice(&generalized_forces[..3]);
        let local_force = world_to_base * Vector3::from_column_slice(&generalized_forces[3..6]);
        pinocchio_force[..3].copy_from_slice(local_force.as_slice());
        pinocchio_force[3..6].copy_from_slice(local_torque.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            pinocchio_force[mapping.velocity_index.expect("active joint")] =
                generalized_forces[6 + joint];
        }
        let acceleration =
            self.aba_with_loads_raw(configuration, velocity, &pinocchio_force, loads);
        self.floating_acceleration_order(&acceleration, base, base_velocity)
    }

    fn floating_acceleration_order(
        &self,
        pinocchio_acceleration: &[f64],
        base: &Frame,
        base_velocity: Twist,
    ) -> Vec<f64> {
        let world_to_base = base.rotation.inverse();
        let local_linear_velocity = world_to_base * base_velocity.linear;
        let local_angular_velocity = world_to_base * base_velocity.angular;
        let local_linear_acceleration = Vector3::from_column_slice(&pinocchio_acceleration[..3]);
        let local_angular_acceleration = Vector3::from_column_slice(&pinocchio_acceleration[3..6]);
        let world_angular_acceleration = base.rotation * local_angular_acceleration;
        let world_linear_acceleration = base.rotation
            * (local_linear_acceleration + local_angular_velocity.cross(&local_linear_velocity));
        let mut output = vec![0.0; 6 + self.joint_mappings.len()];
        output[..3].copy_from_slice(world_angular_acceleration.as_slice());
        output[3..6].copy_from_slice(world_linear_acceleration.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            output[6 + joint] =
                pinocchio_acceleration[mapping.velocity_index.expect("active joint")];
        }
        output
    }

    pub fn mass_matrix(&mut self, configuration: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size * self.velocity_size];
        // SAFETY: the output has `model.nv * model.nv` elements, column-major.
        unsafe {
            dynibo_pinocchio_mass_matrix_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_square_order(&pinocchio)
    }

    pub fn floating_mass_matrix(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size * self.velocity_size];
        unsafe {
            dynibo_pinocchio_mass_matrix_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        let transformations = self.floating_velocity_transform(base);
        let size = transformations.len();
        let mut output = vec![0.0; size * size];
        for column in 0..size {
            for row in 0..size {
                let mut value = 0.0;
                for &(pin_row, row_scale) in &transformations[row] {
                    for &(pin_column, column_scale) in &transformations[column] {
                        value += row_scale
                            * pinocchio[pin_column * self.velocity_size + pin_row]
                            * column_scale;
                    }
                }
                output[column * size + row] = value;
            }
        }
        output
    }

    pub fn floating_gravity(&mut self, configuration: &[f64], base: &Frame) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size];
        unsafe {
            dynibo_pinocchio_gravity_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.floating_generalized_order(&pinocchio, base)
    }

    pub fn coriolis_matrix(&mut self, configuration: &[f64], velocity: &[f64]) -> Vec<f64> {
        let mut pinocchio = vec![0.0; self.velocity_size * self.velocity_size];
        // SAFETY: the output has `model.nv * model.nv` elements, column-major.
        unsafe {
            dynibo_pinocchio_coriolis_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_square_order(&pinocchio)
    }

    pub fn rnea_with_link_load(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        load: Wrench,
    ) -> Vec<f64> {
        let load = [
            load.torque.x,
            load.torque.y,
            load.torque.z,
            load.force.x,
            load.force.y,
            load.force.z,
        ];
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: all buffers match the context dimensions, and the load has six elements.
        unsafe {
            dynibo_pinocchio_rnea_with_link_load_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                load.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        self.dynibo_joint_order(&pinocchio)
    }

    pub fn floating_rnea(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        base: &Frame,
        base_velocity: Twist,
        base_acceleration: Twist,
    ) -> Vec<f64> {
        let base_velocity = [
            base_velocity.angular.x,
            base_velocity.angular.y,
            base_velocity.angular.z,
            base_velocity.linear.x,
            base_velocity.linear.y,
            base_velocity.linear.z,
        ];
        let base_acceleration = [
            base_acceleration.angular.x,
            base_acceleration.angular.y,
            base_acceleration.angular.z,
            base_acceleration.linear.x,
            base_acceleration.linear.y,
            base_acceleration.linear.z,
        ];
        let rotation = base.rotation.coords;
        let mut pinocchio = vec![0.0; self.velocity_size];
        // SAFETY: all state and pose buffers match the free-flyer context dimensions.
        unsafe {
            dynibo_pinocchio_floating_rnea_values(
                self.pointer.as_ptr(),
                configuration.as_ptr(),
                velocity.as_ptr(),
                acceleration.as_ptr(),
                base.translation.vector.as_ptr(),
                rotation.as_ptr(),
                base_velocity.as_ptr(),
                base_acceleration.as_ptr(),
                pinocchio.as_mut_ptr(),
            )
        };
        let mut output = vec![0.0; 6 + self.joint_mappings.len()];
        let local_force = Vector3::new(pinocchio[0], pinocchio[1], pinocchio[2]);
        let local_torque = Vector3::new(pinocchio[3], pinocchio[4], pinocchio[5]);
        let world_torque = base.rotation * local_torque;
        let world_force = base.rotation * local_force;
        output[..3].copy_from_slice(world_torque.as_slice());
        output[3..6].copy_from_slice(world_force.as_slice());
        for (joint, mapping) in self.joint_mappings.iter().enumerate() {
            if let Some(index) = mapping.velocity_index {
                output[6 + joint] = pinocchio[index];
            }
        }
        output
    }

    pub fn floating_rnea_with_loads(
        &mut self,
        configuration: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        base: &Frame,
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let pinocchio = self.rnea_with_loads_raw(configuration, velocity, acceleration, loads);
        self.floating_generalized_order(&pinocchio, base)
    }

    pub fn floating_gravity_with_loads(
        &mut self,
        configuration: &[f64],
        base: &Frame,
        loads: &[PinocchioLoad],
    ) -> Vec<f64> {
        let zero = vec![0.0; self.velocity_size];
        self.floating_rnea_with_loads(configuration, &zero, &zero, base, loads)
    }

    pub fn floating_coriolis_from_rnea(
        &mut self,
        q: &[f64],
        qd: &[f64],
        base: &Frame,
        base_velocity: Twist,
    ) -> Vec<f64> {
        let size = 6 + q.len();
        let mut output = vec![0.0; size * size];
        let zero = vec![0.0; q.len()];
        for column in 0..size {
            let mut plus_qd = qd.to_vec();
            let mut plus_base = base_velocity;
            if column < 3 {
                plus_base.angular[column] += 1.0;
            } else if column < 6 {
                plus_base.linear[column - 3] += 1.0;
            } else {
                plus_qd[column - 6] += 1.0;
            }
            let (plus_q, plus_v, plus_a) = self.state(q, &plus_qd, &zero);
            let plus =
                self.floating_rnea(&plus_q, &plus_v, &plus_a, base, plus_base, Twist::zeros());

            let mut minus_qd = qd.to_vec();
            let mut minus_base = base_velocity;
            if column < 3 {
                minus_base.angular[column] -= 1.0;
            } else if column < 6 {
                minus_base.linear[column - 3] -= 1.0;
            } else {
                minus_qd[column - 6] -= 1.0;
            }
            let (minus_q, minus_v, minus_a) = self.state(q, &minus_qd, &zero);
            let minus = self.floating_rnea(
                &minus_q,
                &minus_v,
                &minus_a,
                base,
                minus_base,
                Twist::zeros(),
            );
            for row in 0..size {
                output[column * size + row] = 0.25 * (plus[row] - minus[row]);
            }
        }
        output
    }
}
