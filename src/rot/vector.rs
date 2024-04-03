use nalgebra::Vector3;

use crate::Vector;

use super::so3;

/// so3 vector representation
/// ```
/// vec3 = [x, y, z]
/// ```
#[derive(Debug)]
pub struct Vec3<T> {
    pub(crate) vector: Vector3<T>,
}

impl<T> Vec3<T>
{
    /// Create a new Vec3
    /// ```
    /// vec3 = [x, y, z]
    /// ```
    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            vector: Vector3::new(x, y, z),
        }
    }
}

impl<T> Vector for Vec3<T>
where
    T: Copy,
{
    type Algebra = so3<T>;

    fn hat(&self) -> Self::Algebra {
        so3 {
            vector: self.vector,
        }
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
            vector: Vector3::new(0., 0., FRAC_PI_2),
        };
        let so3 = v.hat();
        assert_relative_eq!(so3.vector, Vector3::new(0., 0., FRAC_PI_2));
    }
}
