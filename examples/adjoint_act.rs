use std::f64::consts::FRAC_PI_2;

use liealg::prelude::*;
use liealg::{Vec3, Vec6};

fn main() {
    let vec3 = Vec3::new(0., 0., 1.) * FRAC_PI_2;
    let so3 = vec3.hat();
    let adj = so3.exp().adjoint();
    println!("{:.2}", adj);
    println!("{:?}", adj.act(&so3));

    let vec6 = Vec6::new([0., 0., 1.], [0., -1., 0.]) * FRAC_PI_2;
    let se3 = vec6.hat();
    let adj = se3.exp().adjoint();
    println!("{:.2}", adj);
    println!("{:?}", adj.act(&se3));
}
