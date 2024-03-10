//! # Lie group and Lie algebra
//!
//! |manifold|algebra|vector|
//! |-|-|-|
//! |SO3|so3|Vector3|
//! |SE3|se3|Vector6|

#![deny(missing_docs)]
#![deny(missing_debug_implementations)]
#![deny(missing_copy_implementations)]
#![no_std]

mod point;
mod rigid;
mod rot;
mod utils;

pub use point::Point;
pub use rot::{so3, AdjSO3, Vec3, SO3};
pub use rigid::{se3, AdjSE3, Vec6, SE3};

/// lie algebra vector representation
pub trait Vector {
    /// algebra type
    type Algebra;

    /// map vector to algebra
    fn hat(&self) -> Self::Algebra;
}

/// lie algebra trait
pub trait Algebra {
    /// group type
    type Group;

    /// vector type
    type Vector;

    /// Exp map: map algebra to group
    fn exp(&self) -> Self::Group;

    /// map algebra to vector
    fn vee(&self) -> Self::Vector;
}

/// lie algebra trait
pub trait Adjoint {
    ///  dalgebra type
    type Algebra;
    /// map algebra to another algebra
    fn act(&self, other: &Self::Algebra) -> Self::Algebra;
}

/// Group trait
pub trait Group {
    /// algebra type
    type Algebra;
    /// log map, map group to algebra
    fn log(&self) -> Self::Algebra;

    /// adjoint type
    type Adjoint;
    /// create adjoint
    fn adjoint(&self) -> Self::Adjoint;

    /// matrix inverse
    fn inv(&self) -> Self;
    /// matrix multiplication
    fn mat_mul(&self, other: &Self) -> Self;

    /// point type
    type Point;
    /// matrix action on point
    fn act(&self, other: &Self::Point) -> Self::Point;
}
