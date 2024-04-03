use std::f32::consts::FRAC_PI_2;

use liealg::prelude::*;
use liealg::Vec6;

fn main() {
    let vec = Vec6::new([0., 0., 1.], [0., -1., 0.]) * FRAC_PI_2;
    println!("vec: {}", vec);
    let se3 = vec.hat();
    println!("se3: {}", se3);
    let rigid = se3.exp();
    println!("rigid: {:.2}", rigid);
    let se3_ = rigid.log();
    let vec_ = se3_.vee();
    println!("vec_: {}", vec_);
}