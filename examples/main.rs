use liealg::*;

fn main() {
    let r = [0.0, 0.0, 1.0];
    let v = [1.0, 0.0, 0.0];
    let vec = Vec6::<_, 0, 1>::new(r, v).hat();
    let m1 = vec.exp();
    println!("{:?}", m1);
}