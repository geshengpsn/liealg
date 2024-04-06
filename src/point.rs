use core::fmt::Display;

use nalgebra::Vector3;

use crate::Real;

/// point in 3D space
#[derive(Debug)]
pub struct Point<T> {
    pub(crate) val: Vector3<T>,
}

impl<T> Display for Point<T>
where
    T: Display + Real,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.val.fmt(f)
    }
}

impl<T> Point<T> {
    /// constructe a new point
    pub fn new(x: T, y: T, z: T) -> Self {
        Self {
            val: Vector3::new(x, y, z),
        }
    }
}

impl<T: Copy> Point<T> {
    /// get the x coordinate
    pub fn x(&self) -> T {
        self.val[0]
    }

    /// get the y coordinate
    pub fn y(&self) -> T {
        self.val[1]
    }

    /// get the z coordinate
    pub fn z(&self) -> T {
        self.val[2]
    }
}
