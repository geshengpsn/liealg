use core::{
    fmt::{Display, Formatter},
    ops::Mul,
};

use nalgebra::Vector3;

use crate::{Real, Vector};

use super::so3;

/// so3 vector representation
/// ```ignore
/// vec3 = [x, y, z]
/// ```
#[derive(Debug)]
pub struct Vec3<T> {
    pub(crate) val: Vector3<T>,
}

impl<T> Display for Vec3<T>
where
    T: Real + Display,
{
    fn fmt(&self, f: &mut Formatter) -> core::fmt::Result {
        self.val.fmt(f)
    }
}

impl<T> Mul<T> for Vec3<T>
where
    T: Real,
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            val: self.val * rhs,
        }
    }
}

impl<T> Vec3<T> {
    /// Create a new Vec3
    /// ```ignore
    /// vec3 = [x, y, z]
    /// ```
    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            val: Vector3::new(x, y, z),
        }
    }
}

impl<T: Copy> Vec3<T> {
    /// get the vector
    pub fn get(&self) -> [T; 3] {
        [self.val[0], self.val[1], self.val[2]]
    }
}

impl<T> Vector for Vec3<T>
where
    T: Copy,
{
    type Algebra = so3<T>;

    fn hat(&self) -> Self::Algebra {
        so3 { val: self.val }
    }
}

#[cfg(test)]
mod test {
    use core::f64::consts::FRAC_PI_2;

    use approx::assert_relative_eq;

    use super::*;
    #[test]
    fn vector_hat() {
        let v = Vec3 {
            val: Vector3::new(0., 0., FRAC_PI_2),
        };
        let so3 = v.hat();
        assert_relative_eq!(so3.val, Vector3::new(0., 0., FRAC_PI_2));
    }
}
