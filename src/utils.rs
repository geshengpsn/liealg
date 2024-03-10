
use nalgebra::{Matrix3, RealField, Vector3};

pub(crate) fn approx_zero<T: RealField>(v: T) -> bool {
    v < T::default_epsilon()
}

pub(crate) fn approx_zero_vec<T: RealField>(vec: &Vector3<T>) -> bool {
    vec[0] < T::default_epsilon() && vec[1] < T::default_epsilon() && vec[2] < T::default_epsilon()
}

pub(crate) fn axis_angle<T: RealField + Copy>(v: &Vector3<T>) -> (Vector3<T>, T) {
    let angle = v.norm();
    (v / angle, angle)
}

pub(crate) fn hat<T>(v: &Vector3<T>) -> Matrix3<T>
where
    T: RealField + Copy,
{
    let zero = T::zero();
    Matrix3::new(
        zero, -v[2], v[1], 
        v[2], zero, -v[0], 
        -v[1], v[0], zero
    )
}
