use core::ops::Mul;

use nalgebra::{RealField, Vector3, Vector6};

use crate::Vector;

use super::se3;

/// se3 vector representation
#[derive(Debug)]
pub struct Vec6<T> {
    pub(crate) val: Vector6<T>,
}

impl<T> Mul<T> for Vec6<T>
where
    T: Copy + RealField,
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            val: self.val * rhs,
        }
    }
}

impl<T> Vec6<T>
where
    T: Copy,
{
    /// Create a new Vec6 from a rotation and translation
    pub fn new(r: [T; 3], p: [T; 3]) -> Self {
        Self {
            val: Vector6::new(r[0], r[1], r[2], p[0], p[1], p[2]),
        }
    }

    pub(crate) fn r(&self) -> Vector3<T> {
        Vector3::new(self.val[0], self.val[1], self.val[2])
    }

    pub(crate) fn p(&self) -> Vector3<T> {
        Vector3::new(self.val[3], self.val[4], self.val[5])
    }
}

impl<T> Vector for Vec6<T>
where
    T: Copy,
{
    type Algebra = se3<T>;

    fn hat(&self) -> Self::Algebra {
        Self::Algebra { val: self.val }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_new() {
        let r = [1.0, 2.0, 3.0];
        let v = [4.0, 5.0, 6.0];
        let vec = Vec6::new(r, v);
        assert_eq!(vec.val, Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
    }

    #[test]
    fn test_r() {
        let r = [1.0, 2.0, 3.0];
        let v = [4.0, 5.0, 6.0];
        let vec = Vec6::new(r, v);
        assert_eq!(vec.r(), Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn test_p() {
        let r = [1.0, 2.0, 3.0];
        let v = [4.0, 5.0, 6.0];
        let vec = Vec6::new(r, v);
        assert_eq!(vec.p(), Vector3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn test_hat() {
        let r = [1.0, 2.0, 3.0];
        let v = [4.0, 5.0, 6.0];
        let vec = Vec6::new(r, v);
        let algebra = vec.hat();
        assert_eq!(algebra.val, vec.val);
    }
}
