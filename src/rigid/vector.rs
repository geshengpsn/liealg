use core::{
    fmt::{self, Display, Formatter},
    ops::Mul,
};

use nalgebra::Vector6;

use crate::{Real, Vector};

use super::se3;

/// se3 vector representation
#[derive(Debug)]
pub struct Vec6<T> {
    pub(crate) val: Vector6<T>,
}

impl<T> AsRef<Vector6<T>> for Vec6<T> {
    fn as_ref(&self) -> &Vector6<T> {
        &self.val
    }
}

impl<T> Display for Vec6<T>
where
    T: Real + Display,
{
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        self.val.fmt(f)
    }
}

impl<T> Mul<T> for Vec6<T>
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

impl<T> Mul<T> for &Vec6<T>
where
    T: Real,
{
    type Output = Vec6<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Vec6 {
            val: self.val * rhs,
        }
    }
}

impl<T> Mul<&T> for Vec6<T>
where
    T: Real,
{
    type Output = Vec6<T>;

    fn mul(self, rhs: &T) -> Self::Output {
        Vec6 {
            val: self.val * *rhs,
        }
    }
}

impl<T> Mul<&T> for &Vec6<T>
where
    T: Real,
{
    type Output = Vec6<T>;

    fn mul(self, rhs: &T) -> Self::Output {
        Vec6 {
            val: self.val * *rhs,
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
    /// get the array representation of the vector
    pub fn as_array(&self) -> [T; 6] {
        [
            self.val[0],
            self.val[1],
            self.val[2],
            self.val[3],
            self.val[4],
            self.val[5],
        ]
    }
    /// get the array representation of the vector
    pub fn as_slice(&self) -> &[T] {
        self.val.as_slice()
    }

    /// get rotation part of the vector
    pub fn r(&self) -> [T; 3] {
        [self.val[0], self.val[1], self.val[2]]
    }

    /// get translation part of the vector
    pub fn p(&self) -> [T; 3] {
        [self.val[3], self.val[4], self.val[5]]
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
        assert_eq!(vec.r(), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_p() {
        let r = [1.0, 2.0, 3.0];
        let v = [4.0, 5.0, 6.0];
        let vec = Vec6::new(r, v);
        assert_eq!(vec.p(), [4.0, 5.0, 6.0]);
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
