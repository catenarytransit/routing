pub static TEN_TO_14: f64 = 100000000000000.0;

pub fn coord_to_int(x: f64, y: f64) -> (i64, i64) {
    let x = (x * TEN_TO_14) as i64;
    let y = (y * TEN_TO_14) as i64;
    (x, y)
}
