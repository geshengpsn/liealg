use nalgebra::{Matrix3, Vector3};

use crate::{point::Point, utils::approx_zero, Group, Real};

use super::{so3, AdjSO3};

/// SO3 group (rotation matrix), rotation in 3D space
#[derive(Debug)]
pub struct SO3<T> {
    pub(crate) val: Matrix3<T>,
}

impl<T> Group for SO3<T>
where
    T: Real + Copy,
{
    type Algebra = so3<T>;

    fn log(&self) -> Self::Algebra {
        let rot = self.val;
        let one: T = T::one();
        let two = one + one;
        let cos = (rot.trace() - one) / two;
        if cos >= one {
            so3 {
                vector: Vector3::zeros(),
            }
        } else if cos <= -one {
            let res;
            if approx_zero(one + rot[(2, 2)]) {
                res = Vector3::from_column_slice(&[rot[(0, 2)], rot[(1, 2)], one + rot[(2, 2)]])
                    / (two * (one + rot[(2, 2)])).sqrt();
            } else if approx_zero(one + rot[(1, 1)]) {
                res = Vector3::from_column_slice(&[rot[(0, 1)], one + rot[(1, 1)], rot[(2, 1)]])
                    / (two * (one + rot[(1, 1)])).sqrt();
            } else {
                res = Vector3::from_column_slice(&[rot[(0, 0)], rot[(1, 0)], one + rot[(2, 0)]])
                    / (two * (one + rot[(0, 0)])).sqrt();
            }
            so3 {
                vector: res * T::PI(),
            }
        } else {
            let theta = cos.acos();
            let a = (rot - rot.transpose()) * (theta / two / theta.sin());
            so3 {
                vector: Vector3::new(a[(2, 1)], a[(0, 2)], a[(1, 0)]),
            }
        }
    }

    type Adjoint = AdjSO3<T>;

    fn adjoint(&self) -> Self::Adjoint {
        AdjSO3 { val: self.val }
    }

    fn inv(&self) -> Self {
        Self {
            val: self.val.transpose(),
        }
    }

    fn mat_mul(&self, other: &Self) -> Self {
        Self {
            val: self.val * other.val,
        }
    }

    type Point = Point<T>;
    fn act(&self, other: &Self::Point) -> Self::Point {
        Self::Point {
            val: self.val * other.val,
        }
    }
}

#[cfg(test)]
mod test {
    use core::f64::consts::FRAC_PI_2;

    use approx::assert_relative_eq;

    use crate::{rot::Vec3, Vector};

    use super::*;

    #[test]
    fn test_log() {
        let rot = SO3 {
            val: Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.),
        };
        let so3 = rot.log();
        let v = Vec3 {
            vector: Vector3::new(0., 0., FRAC_PI_2),
        };
        assert_relative_eq!(v.hat().vector, so3.vector);
    }

    #[test]
    fn test_adjoint() {
        let rot = SO3 {
            val: Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.),
        };
        let adj = rot.adjoint();
        assert_relative_eq!(adj.val, rot.val);
    }

    #[test]
    fn test_inv() {
        let rot = SO3 {
            val: Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.),
        };
        let inv = rot.inv();
        assert_relative_eq!(inv.val, &rot.val.transpose());
    }

    #[test]
    fn test_mat_mul() {
        let rot1 = SO3 {
            val: Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.),
        };
        let rot2 = SO3 {
            val: Matrix3::identity(),
        };
        let rot3 = rot1.mat_mul(&rot2);
        assert_relative_eq!(rot3.val, &Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.));
    }

    #[test]
    fn test_act() {
        let rot = SO3 {
            val: Matrix3::new(0., -1., 0., 1., 0., 0., 0., 0., 1.),
        };
        let v = Point {
            val: Vector3::new(1., 2., 3.),
        };
        let v_rot = rot.act(&v);
        assert_relative_eq!(v_rot.val, Vector3::new(-2., 1., 3.));
    }
}
