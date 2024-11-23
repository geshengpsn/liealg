use core::{
    fmt::Display,
    ops::{Add, Mul},
};

use nalgebra::{Matrix3, Matrix4, Vector3, Vector6};

use crate::{
    so3,
    utils::{approx_zero, axis_angle, hat, hat_se3},
    Algebra, Real,
};

use super::{Vec6, SE3};

/// se3 group
/// ```ignore
/// se3 = [
///  0 -wz wy x
///  wz 0 -wx y
/// -wy wx 0  z
///  0  0  0  0
/// ]
/// ```
#[derive(Debug)]
#[allow(non_camel_case_types)]
pub struct se3<T> {
    pub(crate) val: Vector6<T>,
}

impl<T> se3<T>
where
    T: Real,
{
    /// Create a new se3 from rotation and translation
    pub fn new(r: [T; 3], p: [T; 3]) -> Self {
        Self {
            val: Vector6::new(r[0], r[1], r[2], p[0], p[1], p[2]),
        }
    }

    /// Create a new identity se3
    pub fn identity() -> Self {
        Self {
            val: Vector6::zeros(),
        }
    }
}

impl<T> Display for se3<T>
where
    T: Real + Display,
{
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        hat_se3(&self.val).fmt(f)
    }
}

impl<T> Mul<T> for se3<T>
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

impl<T> Mul<&T> for se3<T>
where
    T: Real,
{
    type Output = Self;

    fn mul(self, rhs: &T) -> Self::Output {
        Self {
            val: self.val * *rhs,
        }
    }
}

impl<T> Add for se3<T>
where
    T: Real,
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            val: self.val + rhs.val,
        }
    }
}

impl<T> Algebra for se3<T>
where
    T: Real,
{
    type Group = SE3<T>;
    type Vector = Vec6<T>;

    fn exp(&self) -> Self::Group {
        let vec = self.vee();
        let v = Vector3::from_column_slice(&vec.p());
        let w = Vector3::from_column_slice(&vec.r());
        let (axis, theta) = axis_angle(&w);
        if approx_zero(theta) {
            let mut res = Matrix4::identity();
            res.view_mut((0, 3), (3, 1)).copy_from(&v);
            SE3 { val: res }
        } else {
            let mut res = Matrix4::identity();
            let w_so3 = hat(&axis);
            let vv = Matrix3::identity() * theta
                + w_so3 * (T::one() - theta.cos())
                + w_so3 * w_so3 * (theta - theta.sin());
            let rot = so3 { val: w }.exp();
            let rot = rot.val;
            res.view_mut((0, 0), (3, 3)).copy_from(&rot);
            res.view_mut((0, 3), (3, 1)).copy_from(&(vv * v / theta));
            SE3 { val: res }
        }
    }

    fn vee(&self) -> Self::Vector {
        Vec6::<_> { val: self.val }
    }
}

#[cfg(test)]
mod test {
    use core::f64::consts::FRAC_PI_2;

    use approx::assert_relative_eq;
    use nalgebra::Matrix4;

    use crate::{rigid::Vec6, Vector};

    use super::*;

    #[test]
    fn se3_exp() {
        //      | z
        //      |
        // x ---+
        //       \
        //        y
        let se3 = Vec6::new([0., 0., 1.], [1., 0., 0.]).hat() * FRAC_PI_2;
        let v = se3.exp();

        let s = SE3 {
            val: Matrix4::new(
                0., -1., 0., 1., 1., 0., 0., 1., 0., 0., 1., 0., 0., 0., 0., 1.,
            ),
        };
        assert_relative_eq!(v.val, s.val);
    }
}
