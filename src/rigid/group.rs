use nalgebra::{Matrix3, Matrix4, Matrix6, Vector3, Vector4, Vector6};

use crate::{
    utils::{approx_zero_vec, axis_angle, hat},
    Group, Point, Real, SO3,
};

use super::{se3, AdjSE3};

/// SE3 group, rotation and translation in 3D space
///
/// SE3 is a 4x4 matrix
/// ```ignore
/// SE3 = [
///  R t
///  0 1
/// ]
#[derive(Debug)]
pub struct SE3<T> {
    pub(crate) val: Matrix4<T>,
}

impl<T> SE3<T>
where
    T: Real,
{
    /// Create a new SE3 from a SO3 and translation
    pub fn new(rot: &SO3<T>, p: [T; 3]) -> Self {
        Self::from_rp(&rot.val, &Vector3::from(p))
    }

    /// create SO3 and translation from SE3
    pub fn rot_trans(&self) -> (SO3<T>, [T; 3]) {
        let (r, p) = self.rp();
        (SO3 { val: r }, p.into())
    }
}

impl<T> SE3<T>
where
    T: Real,
{
    fn rp(&self) -> (Matrix3<T>, Vector3<T>) {
        let r = self.val.fixed_view::<3, 3>(0, 0);
        let p = self.val.fixed_view::<3, 1>(0, 3);
        (r.into(), p.into())
    }

    fn from_rp(r: &Matrix3<T>, p: &Vector3<T>) -> Self {
        let mut val = Matrix4::identity();
        val.fixed_view_mut::<3, 3>(0, 0).copy_from(r);
        val.fixed_view_mut::<3, 1>(0, 3).copy_from(p);
        Self { val }
    }
}

impl<T> Group for SE3<T>
where
    T: Real,
{
    type Algebra = se3<T>;

    fn log(&self) -> Self::Algebra {
        let (r, p) = self.rp();
        let w = SO3 { val: r }.log().vector;

        if approx_zero_vec(&w) {
            let mut res = Vector6::zeros();
            res.view_mut((3, 0), (3, 1)).copy_from(&p);
            Self::Algebra { val: res }
        } else {
            let two = T::one() + T::one();
            let (w_, theta) = axis_angle(&w);
            let w_so3 = hat(&w_);
            let v = {
                let a = Matrix3::identity() / theta;
                let b = w_so3 / two;
                let c = T::one() / theta - T::one() / (theta / two).tan() / two; // * w_so3 * w_so3;
                let c = w_so3 * w_so3 * c;
                (a - b + c) * p
            };
            let mut res = Vector6::zeros();
            res.view_mut((0, 0), (3, 1)).copy_from(&w);
            res.view_mut((3, 0), (3, 1)).copy_from(&(v * theta));
            Self::Algebra { val: res }
        }
    }

    type Adjoint = AdjSE3<T>;

    fn adjoint(&self) -> Self::Adjoint {
        let (r, p) = self.rp();
        let p_so3 = hat(&p);
        let mut res = Matrix6::zeros();
        res.view_mut((0, 0), (3, 3)).copy_from(&r);
        res.view_mut((3, 0), (3, 3)).copy_from(&(p_so3 * r));
        res.view_mut((3, 3), (3, 3)).copy_from(&r);
        Self::Adjoint { val: res }
    }

    fn inv(&self) -> Self {
        let (r, p) = self.rp();
        Self::from_rp(&r.transpose(), &(-r.transpose() * p))
    }

    fn mat_mul(&self, other: &Self) -> Self {
        SE3 {
            val: self.val * other.val,
        }
    }

    type Point = Point<T>;

    fn act(&self, other: &Self::Point) -> Self::Point {
        let p4 = self.val * Vector4::new(other.val.x, other.val.y, other.val.z, T::one());
        Point {
            val: Vector3::new(p4.x, p4.y, p4.z),
        }
    }
}

#[cfg(test)]
mod test {
    use core::f64::consts::FRAC_PI_2;

    use approx::assert_relative_eq;

    use crate::Algebra;

    use super::*;

    #[test]
    fn test_new() {
        let se3 = SE3::<f64> {
            val: Matrix4::identity(),
        };
        assert_eq!(se3.val, Matrix4::identity());
    }

    #[test]
    fn test_rp() {
        let se3 = SE3 {
            val: Matrix4::new(
                1., 0., 0., 1., 0., 1., 0., 2., 0., 0., 1., 3., 0., 0., 0., 1.,
            ),
        };
        let (r, p) = se3.rp();
        assert_eq!(r, Matrix3::new(1., 0., 0., 0., 1., 0., 0., 0., 1.));
        assert_eq!(p, Vector3::new(1., 2., 3.));
    }

    #[test]
    fn test_from_rp() {
        let se3 = SE3::from_rp(
            &Matrix3::new(1., 0., 0., 0., 1., 0., 0., 0., 1.),
            &Vector3::new(1., 2., 3.),
        );
        assert_eq!(
            se3.val,
            Matrix4::new(1., 0., 0., 1., 0., 1., 0., 2., 0., 0., 1., 3., 0., 0., 0., 1.)
        );
    }

    #[test]
    fn test_log() {
        let se3 = SE3::from_rp(
            &Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.),
            &Vector3::new(1., 1., 0.),
        )
        .log()
        .vee();
        assert_relative_eq!(se3.val, Vector6::new(0., 0., 1., 1., 0., 0.) * FRAC_PI_2);
    }

    #[test]
    fn test_adjoint() {
        let se3 = SE3::from_rp(
            &Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.),
            &Vector3::new(1., 1., 0.),
        )
        .adjoint();
        assert_relative_eq!(
            se3.val,
            Matrix6::new(
                0., -1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,
                1., 0., -1., 0., 0., 0., -1., 1., 0., 0., 1., 1., 0., 0., 0., 1.,
            )
        );
    }

    #[test]
    fn test_inv() {
        let se3 = SE3::from_rp(
            &Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.),
            &Vector3::new(1., 1., 0.),
        );
        assert_relative_eq!(se3.mat_mul(&se3.inv()).val, Matrix4::identity());
    }
}
