pub static TEN_TO_14: f64 = 100000000000000.0;

pub fn coord_to_int(x: f64, y: f64) -> (i64, i64) {
    let x = (x * TEN_TO_14) as i64;
    let y = (y * TEN_TO_14) as i64;
    (x, y)
}

pub fn int_to_coord(x: i64, y: i64) -> (f64, f64) {
    let x = x as f64 / TEN_TO_14;
    let y = y as f64 / TEN_TO_14;
    (x, y)
}
