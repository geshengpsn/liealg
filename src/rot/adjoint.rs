use core::fmt::Display;

use nalgebra::Matrix3;

use crate::Adjoint;
use crate::Real;
use crate::Vector;

use super::so3;
use super::Vec3;

/// Adjoint of SO3
///
/// AdjSO3 = SO3
#[derive(Debug)]
pub struct AdjSO3<T> {
    pub(crate) val: Matrix3<T>,
}

impl<T> Display for AdjSO3<T>
where
    T: Display + Real,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.val.fmt(f)
    }
}

impl<T> AdjSO3<T>
where
    T: Real,
{
    /// Create a new AdjSO3 from a slice without checking the contents
    ///
    /// # Safety
    /// use ```AdjSO3::new()``` instead if you are not sure about the contents of the slice is a valid adjoint matrix
    pub fn new_unchecked(val: &[T]) -> Self {
        Self {
            val: Matrix3::from_column_slice(val),
        }
    }
}

impl<T> Adjoint for AdjSO3<T>
where
    T: Real,
{
    type Algebra = so3<T>;

    fn act(&self, other: &Self::Algebra) -> Self::Algebra {
        Vec3 {
            val: self.val * other.val,
        }
        .hat()
    }
}

#[cfg(test)]
mod test {
    use nalgebra::Vector3;

    use super::*;

    #[test]
    fn test_adjoint() {
        let adj = AdjSO3::<f64> {
            val: Matrix3::identity(),
        };
        let _ = adj.act(&so3 {
            val: Vector3::new(1., 2., 3.),
        });
    }
}
