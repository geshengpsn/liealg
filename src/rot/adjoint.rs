use nalgebra::{Matrix3, RealField};

use crate::Adjoint;
use crate::Vector;

use super::so3;
use super::Vec3;

/// Adjoint of SO3
#[derive(Debug)]
pub struct AdjSO3<T> {
    pub(crate) val: Matrix3<T>,
}

impl<T> Adjoint for AdjSO3<T>
where
    T: Copy + RealField,
{
    type Algebra = so3<T>;

    fn act(&self, other: &Self::Algebra) -> Self::Algebra {
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
        let adj = AdjSO3::<f64>{
            val: Matrix3::identity()
        };
        let _ = adj.act(&so3 {
            vector: Vector3::new(1., 2., 3.),
        });
    }
}
