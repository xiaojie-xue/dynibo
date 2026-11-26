use super::{Twist, Wrench};

#[inline]
pub(crate) fn motion_cross(lhs: Twist, rhs: Twist) -> Twist {
    Twist::new(
        lhs.angular.cross(&rhs.angular),
        lhs.linear.cross(&rhs.angular) + lhs.angular.cross(&rhs.linear),
    )
}

#[inline]
pub(crate) fn force_cross(motion: Twist, force: Wrench) -> Wrench {
    Wrench::new(
        motion.angular.cross(&force.torque) + motion.linear.cross(&force.force),
        motion.angular.cross(&force.force),
    )
}
