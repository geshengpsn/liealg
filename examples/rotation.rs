use liealg::prelude::*;
use liealg::Vec3;
use std::f64::consts::FRAC_PI_2;

fn main() {
    let vec = Vec3::new(FRAC_PI_2, 0., 0.);
    println!("vec: {}", vec);
    let so3 = vec.hat();
    println!("so3: {:.2}", so3);
    let rot = so3.exp();
    println!("rot: {:.2}", rot);
    let adj = rot.adjoint();
    println!("adj: {:.2}", adj);
    let so3_ = rot.log();
    let vec_ = so3_.vee();
    println!("vec_: {}", vec_);
}
