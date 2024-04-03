use liealg::prelude::*;
use liealg::Vec6;

fn main() {
    let vec = Vec6::new([0., 0., 1.], [0., -1., 0.]);
    println!("vec: {:?}", vec);
    let se3 = vec.hat();
    println!("se3: {:?}", se3);
    let rigid = se3.exp();
    println!("rigid: {:?}", rigid);
    let se3_ = rigid.log();
    let vec_ = se3_.vee();
    println!("vec_: {:?}", vec_);
}