use crate::{se3, so3, AdjSE3, AdjSO3, Point, Vec3, Vec6, SE3, SO3};

macro_rules! impl_clone {
    ($($t:ty)*) => {
        $(
            impl<T: Clone> Clone for $t {
                fn clone(&self) -> Self {
                    Self {
                        val: self.val.clone(),
                    }
                }
            }
        )*
    };
}

impl_clone!(
    Point<T>
    AdjSO3<T> so3<T> SO3<T> Vec3<T>
    AdjSE3<T> se3<T> SE3<T> Vec6<T>
);

#[test]
fn clone_test() {
    use crate::prelude::*;
    let p = Point::new(1., 2., 3.);
    let _ = p.clone();
    let v = Vec3::new(1., 2., 3.);
    let _ = v.clone();
    let so3 = v.hat();
    let _ = so3.clone();
    let mat = so3.exp();
    let _ = mat.clone();
    let adj = mat.adjoint();
    let _ = adj.clone();

    let v = Vec6::new([1., 2., 3.], [4., 5., 6.]);
    let _ = v.clone();
    let se3 = v.hat();
    let _ = se3.clone();
    let mat = se3.exp();
    let _ = mat.clone();
    let adj = mat.adjoint();
    let _ = adj.clone();
}
