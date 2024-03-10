use nalgebra::Vector3;

/// point in 3D space
#[derive(Debug)]
pub struct Point<T, const O: usize> {
    pub(crate) val: Vector3<T>,
}

impl<T, const O: usize> Point<T, O> {
    /// constructe a new point
    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            val: Vector3::new(x, y, z),
        }
    }
}
