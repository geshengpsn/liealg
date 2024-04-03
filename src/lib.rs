//! # Lie group and Lie algebra in rust
//! 
//! liealg is a library for computing Lie algebra and Lie group in 3D space(SO3 and SE3). 
//! It is mainly used in robot kinematics and other related area. 
//! If you want to do some general Lie group and Lie algebra calculations, 
//! it is better not to use this library.
//! 
//! ## Traits
//! liealg provides a set of mathematical entity traits for Lie group and Lie algebra.
//! these are [Adjoint], [Algebra], [Group], [Vector].
//! liealg provides a standard implementation of these traits, but users can also implement their own types.
//!
//! liealg also provides a trait [Real] for using more types of real numbers, except f32, f64, it can support most real number types, such as rational and decimal, users can also define their own real number types.
//! 
//! ## Implementations
//! 
//! liealg provides two implementations of Lie group and Lie algebra, SO3 and SE3, which correspond to rotation and rigid body motion in 3D space.
//! 
//! |group|algebra|vector|
//! |-|-|-|
//! |SO3|so3|Vector3|
//! |SE3|se3|Vector6|
//! 
//! ## Usage
//! ```toml
//! [dependencies]
//! liealg = "0.1"
//! ```
//! 
//! import prelude module
//! ```rust
//! use liealg::prelude::*;
//! ```
//! 
//! ## Example
//! 
//! ### Rotation
//! ```rust
//! use liealg::prelude::*;
//! use liealg::{se3, Vec3};
//! ```
//! 
//! ```rust
//! use liealg::prelude::*;
//! use liealg::{se3, Vec6};
//! let r = [0.0, 0.0, 1.0];
//! let v = [1.0, 0.0, 0.0];
//! let vec = Vec6::new(r, v).hat();
//! let m1 = vec.exp();
//! println!("{:?}", m1);
//! ```
//! 

#![deny(missing_docs)]
#![deny(missing_debug_implementations)]
#![no_std]

mod point;
pub mod rigid;
pub mod rot;
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
