use nalgebra::{Matrix6, RealField};

use crate::Adjoint;

use super::se3;

/// Adjoint of SE3
#[derive(Debug)]
pub struct AdjSE3<T> {
    pub(crate) val: Matrix6<T>,
}

impl<T> Adjoint for AdjSE3<T>
where
    T: Copy + RealField,
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
