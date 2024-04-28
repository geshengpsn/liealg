use crate::{se3, so3, AdjSE3, AdjSO3, Point, Real, Vec3, Vec6, SE3, SO3};
use approx::{AbsDiffEq, RelativeEq};
macro_rules! impl_approx {
    ($($t:ty)*) => {
        $(
            impl<T: PartialEq> PartialEq for $t {
                fn eq(&self, other: &Self) -> bool {
                    self.val == other.val
                }
            }

            impl<T: AbsDiffEq + Real> AbsDiffEq for $t
            where
                T::Epsilon: Copy,
            {
                type Epsilon = T::Epsilon;

                fn default_epsilon() -> Self::Epsilon {
                    T::default_epsilon()
                }

                fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
                    other.val.abs_diff_eq(&self.val, epsilon)
                }
            }

            impl<T: RelativeEq + Real> RelativeEq for $t
            where
                T::Epsilon: Copy,
            {
                fn default_max_relative() -> Self::Epsilon {
                    T::default_max_relative()
                }

                fn relative_eq(
                    &self,
                    other: &Self,
                    epsilon: Self::Epsilon,
                    max_relative: Self::Epsilon,
                ) -> bool {
                    other.val.relative_eq(&self.val, epsilon, max_relative)
                }
            }
        )*
    };
}

impl_approx!(
    Point<T>
    AdjSO3<T> so3<T> SO3<T> Vec3<T>
    AdjSE3<T> se3<T> SE3<T> Vec6<T>
);

#[test]
fn test_approx() {
    use approx::assert_relative_eq;
    assert_ne!(Point::new(0.1 + 0.2, 0., 0.), Point::new(0.3, 0., 0.));
    assert_relative_eq!(Point::new(0.1 + 0.2, 0., 0.), Point::new(0.3, 0., 0.));
}
