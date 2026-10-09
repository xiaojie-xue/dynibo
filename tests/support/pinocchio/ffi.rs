//! Declarations for the C++ reference bridge.

unsafe extern "C" {
    pub(super) fn dynibo_pinocchio_create_for_frame(
        urdf_path: *const std::ffi::c_char,
        frame_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    pub(super) fn dynibo_pinocchio_create_floating_for_frame(
        urdf_path: *const std::ffi::c_char,
        frame_name: *const std::ffi::c_char,
    ) -> *mut std::ffi::c_void;
    pub(super) fn dynibo_pinocchio_destroy(context: *mut std::ffi::c_void);
    pub(super) fn dynibo_pinocchio_dof(context: *const std::ffi::c_void) -> usize;
    pub(super) fn dynibo_pinocchio_configuration_size(context: *const std::ffi::c_void) -> usize;
    pub(super) fn dynibo_pinocchio_neutral_configuration(
        context: *const std::ffi::c_void,
        q: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_joint_configuration_index(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    pub(super) fn dynibo_pinocchio_joint_configuration_dimension(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    pub(super) fn dynibo_pinocchio_joint_velocity_index(
        context: *const std::ffi::c_void,
        joint_name: *const std::ffi::c_char,
    ) -> usize;
    pub(super) fn dynibo_pinocchio_frame_index(
        context: *const std::ffi::c_void,
        frame_name: *const std::ffi::c_char,
    ) -> usize;
    pub(super) fn dynibo_pinocchio_link_frame_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        rotation: *mut f64,
        translation: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_link_jacobian_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        jacobian: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_link_jacobian_derivative_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        derivative: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_link_velocity_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        velocity: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_link_acceleration_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        acceleration: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_gravity_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        gravity: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_rnea_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        torque: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_aba_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        torque: *const f64,
        acceleration: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_aba_with_link_load_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        torque: *const f64,
        load: *const f64,
        acceleration: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_aba_with_loads_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        torque: *const f64,
        frame_indices: *const usize,
        loads: *const f64,
        load_count: usize,
        acceleration: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_mass_matrix_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        mass: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_coriolis_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        coriolis: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_rnea_with_link_load_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        load: *const f64,
        torque: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_rnea_with_loads_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        frame_indices: *const usize,
        loads: *const f64,
        load_count: usize,
        torque: *mut f64,
    );
    pub(super) fn dynibo_pinocchio_floating_rnea_values(
        context: *mut std::ffi::c_void,
        q: *const f64,
        qd: *const f64,
        qdd: *const f64,
        base_translation: *const f64,
        base_rotation_xyzw: *const f64,
        base_velocity: *const f64,
        base_acceleration: *const f64,
        torque: *mut f64,
    );
}
