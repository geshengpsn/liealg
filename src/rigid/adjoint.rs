use core::fmt::Debug;

use nalgebra::Matrix6;

use crate::{Adjoint, Real};

use super::se3;

/// Adjoint of SE3
/// 
/// AdjSE3 is a 6x6 matrix
/// ```
/// AdjSE3 = [
///   R 0
///  tR R
/// ]
/// ```
#[derive(Debug)]
pub struct AdjSE3<T> {
    pub(crate) val: Matrix6<T>,
}

impl<T> AdjSE3<T>
where
    T: Real,
{
    /// Create a new AdjSE3 from a slice without checking the contents
    /// 
    /// # Safety
    /// use ```SE3::adjoint()``` instead if you are not sure the contents of the slice is valid
    pub fn new_unchecked(val: &[T]) -> Self {
        Self { val: Matrix6::from_column_slice(val) }
    }
}

impl<T> Adjoint for AdjSE3<T>
where
    T: Real,
{
    type Algebra = se3<T>;

    fn act(&self, other: &Self::Algebra) -> Self::Algebra {
        se3 {
            val: self.val * other.val,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_new() {
        let adj = AdjSE3 {
            val: Matrix6::<f64>::identity(),
        };
        assert_eq!(adj.val, Matrix6::identity());
    }
}
