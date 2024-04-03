use std::f64::consts::FRAC_PI_2;

use liealg::{prelude::*, Point};
use liealg::{Vec3, Vec6};

fn main() {
    let p = Point::new(1., 2., 3.);

    let rot = (Vec3::new(0., 0., 1.) * FRAC_PI_2).hat().exp();
    println!("new p: {:?}", rot.act(&p));

    let vec6 = Vec6::new([0., 0., 1.], [0., -1., 0.]) * FRAC_PI_2;
    let t = vec6.hat().exp();
    println!("new p: {:?}", t.act(&p));
}