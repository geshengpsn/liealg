use core::ops::Mul;

use super::SO3;
use crate::{
    utils::{approx_zero, axis_angle, hat},
    Algebra, Vec3,
};
use nalgebra::{Matrix3, RealField, Vector3};

/// so3 group
#[allow(non_camel_case_types)]
#[derive(Debug)]
pub struct so3<T> {
    pub(crate) vector: Vector3<T>,
}

impl<T> Mul<T> for so3<T>
where
    T: Copy + RealField,
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        Self {
            vector: self.vector * rhs,
        }
    }
}

impl<T> Algebra for so3<T>
where
    T: RealField + Copy,
{
    type Group = SO3<T>;

    type Vector = Vec3<T>;

    fn exp(&self) -> Self::Group {
        if approx_zero(self.vector.norm()) {
            SO3 {
                val: Matrix3::identity(),
            }
        } else {
            let (w, angle) = axis_angle(&self.vector);
            let w_so3 = hat(&w);
            SO3 {
                val: Matrix3::identity()
                    + w_so3 * angle.sin()
                    + w_so3 * w_so3 * (T::one() - angle.cos()),
            }
        }
    }

    fn vee(&self) -> Self::Vector {
        Vec3 {
            vector: self.vector,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use approx::assert_relative_eq;
    use core::f64::consts::FRAC_PI_2;

    #[test]
    fn so3_exp() {
        let so3 = so3 {
            vector: Vector3::new(0., 0., FRAC_PI_2),
        };
        let rot_mat = so3.exp();
        assert_relative_eq!(
            rot_mat.val,
            &Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.,)
        );
    }

    #[test]
    fn so3_vee() {
        let so3 = so3 {
            vector: Vector3::new(0., 0., FRAC_PI_2),
        };
        let v = so3.vee();
        assert_relative_eq!(v.vector, Vector3::new(0., 0., FRAC_PI_2));
    }
}
