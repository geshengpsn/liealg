use nalgebra::{Matrix3, Vector3};

use crate::Real;

pub(crate) fn approx_zero<T: Real>(v: T) -> bool {
    v < T::epsilon()
}

pub(crate) fn approx_zero_vec<T: Real>(v: &Vector3<T>) -> bool {
    length(v) < T::epsilon()
}

pub(crate) fn axis_angle<T: Real>(v: &Vector3<T>) -> (Vector3<T>, T) {
    let angle = length(v);
    (v / angle, angle)
}

pub(crate) fn length<T: Real>(v: &Vector3<T>) -> T {
    (v.x * v.x + v.y * v.y + v.z * v.z).sqrt()
}

pub(crate) fn hat<T: Real>(v: &Vector3<T>) -> Matrix3<T> {
    let zero = T::zero();
    Matrix3::new(zero, -v[2], v[1], v[2], zero, -v[0], -v[1], v[0], zero)
}
