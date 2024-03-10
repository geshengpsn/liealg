use nalgebra::{Matrix3, RealField};

use crate::Adjoint;
use crate::Vector;

use super::so3;
use super::Vec3;

/// Adjoint of SO3
#[derive(Debug)]
pub struct AdjSO3<T, const S: usize, const B: usize> {
    pub(crate) val: Matrix3<T>,
}

impl<T, const S: usize, const B: usize> Adjoint for AdjSO3<T, S, B>
where
    T: Copy + RealField,
{
    type InPutAlgebra<const O: usize> = so3<T, B, O>;
    type OutPutAlgebra<const O: usize> = so3<T, S, O>;

    fn act<const O: usize>(&self, other: &Self::InPutAlgebra<O>) -> Self::OutPutAlgebra<O> {
        Vec3 {
            vector: self.val * other.vector,
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
        let adj = AdjSO3::<f64, 0, 1>{
            val: Matrix3::identity()
        };
        let _ = adj.act(&so3::<f64, 1, 2> {
            vector: Vector3::new(1., 2., 3.),
        });
    }
}
