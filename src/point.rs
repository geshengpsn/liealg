use nalgebra::Vector3;

/// point in 3D space
#[derive(Debug)]
pub struct Point<T> {
    pub(crate) val: Vector3<T>,
}

impl<T> Point<T> {
    /// constructe a new point
    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            val: Vector3::new(x, y, z),
        }
    }
}
