use std::f64::consts::FRAC_PI_2;
use liealg::prelude::*;
use liealg::Vec3;

fn main() {
    let vec = Vec3::new(FRAC_PI_2, 0., 0.);
    println!("vec: {:?}", vec);
    let so3 = vec.hat();
    println!("so3: {:?}", so3);
    let rot = so3.exp();
    println!("rot: {:?}", rot);
    let so3_ = rot.log();
    let vec_ = so3_.vee();
    println!("vec_: {:?}", vec_);
}
