use liealg::prelude::*;
use liealg::{se3, Vec6};

fn a<T: Real>(a: se3<T>) {}

fn main() {
    let r = [0.0, 0.0, 1.0];
    let v = [1.0, 0.0, 0.0];
    let vec = Vec6::new(r, v).hat();
    let m1 = vec.exp();
    println!("{:?}", m1);
}
