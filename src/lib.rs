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
//! |SO3|so3|Vec3|
//! |SE3|se3|Vec6|
//!
//! ## Usage
//! add liealg to your dependencies
//! ```toml
//! [dependencies]
//! liealg = "0.1"
//! ```
//!
//! import prelude module
//! ```rust
//! use liealg::prelude::*;
//! ```
//! Rotation
//! ```ignore
//! Vec3 ---> so3 ---> SO3 -------> AdjSO3
//!      hat      exp      adjoint
//!
//! Vec3 <--- so3 <--- SO3
//!       vee      log
//! ```
//!
//! Rigid body motion
//! ```ignore
//! Vec6 ---> se3 ---> SE3 -------> AdjSE3
//!      hat      exp      adjoint
//!
//! Vec6 <--- se3 <--- SE3
//!       vee      log
//! ```
//!  
//! ## Example
//! ### Rotation
//! ```rust
//! use std::f32::consts::FRAC_PI_2;
//! use liealg::prelude::*;
//! use liealg::Vec3;
//! let vec = Vec3::new(0., 0., FRAC_PI_2);
//! println!("vec: {}", vec);
//! let so3 = vec.hat();
//! println!("so3: {}", so3);
//! let rot = so3.exp();
//! println!("rot: {:.2}", rot);
//! let so3_ = rot.log();
//! let vec_ = so3_.vee();
//! println!("vec_: {}", vec_);
//! ```
//!
//! ### Rigid body motion
//!
//! ```rust
//! use std::f32::consts::FRAC_PI_2;
//! use liealg::prelude::*;
//! use liealg::Vec6;
//! let vec = Vec6::new([0., 0., 1.], [0., -1., 0.]) * FRAC_PI_2;
//! println!("vec: {}", vec);
//! let se3 = vec.hat();
//! println!("se3: {}", se3);
//! let rigid = se3.exp();
//! println!("rigid: {:.2}", rigid);
//! let se3_ = rigid.log();
//! let vec_ = se3_.vee();
//! println!("vec_: {}", vec_);
//! ```
//!

#![deny(missing_docs)]
#![deny(missing_debug_implementations)]
#![cfg_attr(not(test), no_std)]

mod impl_approx;
mod impl_clone;
mod point;
pub mod rigid;
pub mod rot;
mod utils;

use core::fmt::Debug;

use num_traits::{real::Real as NumReal, FloatConst, NumAssignOps};
pub use point::Point;
pub use rigid::{se3, AdjSE3, Vec6, SE3};
pub use rot::{so3, AdjSO3, Vec3, SO3};
pub use utils::*;

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

    // fn bracket(&self) -> Self::Algebra;
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
