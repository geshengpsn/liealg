use nalgebra::{Matrix6, RealField};

use crate::Adjoint;

use super::se3;

/// Adjoint of SE3
#[derive(Debug)]
pub struct AdjSE3<T, const S: usize, const B: usize> {
    pub(crate) val: Matrix6<T>,
}

impl<T, const S: usize, const B: usize> Adjoint for AdjSE3<T, S, B>
where
    T: RealField + Copy,
{
    type InPutAlgebra<const O: usize> = se3<T, B, O>;

    type OutPutAlgebra<const O: usize> = se3<T, S, O>;

    fn act<const O: usize>(&self, other: &Self::InPutAlgebra<O>) -> Self::OutPutAlgebra<O> {
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
        let adj = AdjSE3::<f64, 0, 1> {
            val: Matrix6::identity(),
        };
        assert_eq!(adj.val, Matrix6::identity());
    }
}
