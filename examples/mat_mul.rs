use std::f64::consts::FRAC_PI_2;

use liealg::{prelude::*, Vec3, Vec6};

fn main() {
    let v = Vec3::new(0., 0., 1.) * FRAC_PI_2;
    let m = v.hat().exp();
    println!("{:.2}", m.mat_mul(&m));

    let v = Vec6::new([0., 0., 1.], [0., -1., 0.]) * FRAC_PI_2;
    let m = v.hat().exp();
    println!("{:.2}", m.mat_mul(&m));
}
