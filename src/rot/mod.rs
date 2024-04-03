//! Rotation in 3D space

mod adjoint;
mod algebra;
mod group;
mod vector;

pub use adjoint::AdjSO3;
pub use algebra::so3;
pub use group::SO3;
pub use vector::Vec3;
