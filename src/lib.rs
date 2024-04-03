//! # Lie group and Lie algebra
//!
//! |manifold|algebra|vector|
//! |-|-|-|
//! |SO3|so3|Vector3|
//! |SE3|se3|Vector6|

#![deny(missing_docs)]
#![deny(missing_debug_implementations)]
#![no_std]

mod point;
mod rigid;
mod rot;
mod utils;

use core::fmt::Debug;

use num_traits::{real::Real as NumReal, FloatConst, NumAssignOps};
pub use point::Point;
pub use rigid::{se3, AdjSE3, Vec6, SE3};
pub use rot::{so3, AdjSO3, Vec3, SO3};

/// prelude module
pub mod prelude {
    pub use crate::{Adjoint, Algebra, Group, Real, Vector};
}
/// # real number trait
/// support ops: +, -, *, /, %, +=, -=, *=, /=, %=
///
/// consts: 0, 1, π, 1/π, ln2, ......
///
/// compare ops: >, <, <=, >=
pub trait Real: NumReal + Debug + NumAssignOps + FloatConst + 'static {}

impl<T> Real for T where T: NumReal + Debug + NumAssignOps + FloatConst + 'static {}

#[test]
fn trait_test() {
    fn a<T: Real>(_: T) {}
    a(1f64);
    a(1f32);
}

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
