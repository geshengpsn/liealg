use std::f64::consts::FRAC_PI_2;

use liealg::prelude::*;
use liealg::{Vec3, Vec6};

fn main() {
    let rot = (Vec3::new(0., 0., 1.) * FRAC_PI_2).hat().exp();
    let inv_rot = rot.inv();
    println!("rot: {:.2}", rot);
    println!("inv_rot: {:.2}", inv_rot);

    let vec6 = Vec6::new([0., 0., 1.], [0., -1., 0.]) * FRAC_PI_2;
    let t = vec6.hat().exp();
    let inv_t = t.inv();
    println!("t: {:.2}", t);
    println!("inv_t: {:.2}", inv_t);
}
