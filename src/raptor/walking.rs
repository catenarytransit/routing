
use ahash::AHashMap;
use kd_tree::KdPoint;
use kd_tree::KdTree;
#[derive(Debug, Clone, Copy, PartialEq)]
struct Station(f64, f64, f64, i64);

impl KdPoint for Station {
    type Scalar = f64;
    type Dim = typenum::U3;
    fn at(&self, i: usize) -> Self::Scalar {
        match i {
            0 => self.0,
            1 => self.1,
            2 => self.2,
            _ => 0.0,
        }
    }
    fn dim() -> usize {
        3
    }
}
impl Station {
    fn dist(&self, other: Station) -> f64 {
        let dists = [self.0 - other.0, self.1 - other.1, self.2 - other.2];
        (dists[0] * dists[0] + dists[1] * dists[1] + dists[2] * dists[2]).sqrt()
    }
}
/// Generate walking paths with a maximum distance in meters.
/// Takes an iterator of stations with (id,lat,lon) measured in 10^-14 degrees
pub fn generate_walking_paths(
    stations: &[(i64, i64, i64)],
    max_dist_m: i64,
) -> AHashMap<i64, Vec<(i64, i32)>> {
    const EARTH_RADIUS_M: f64 = 6_371_000.0;
    const INT_TO_RAD: f64 = std::f64::consts::PI / 180.0 / 1.0e14;
    let mut points: Vec<Station> = Vec::with_capacity(stations.len());
    for (id, lat, long) in stations.iter().copied() {
        let lat_rad = (lat as f64).to_radians() / 1.0e14;
        let long_rad = (long as f64).to_radians() / 1.0e14;
        points.push(Station(
            lat_rad.cos() * long_rad.sin(),
            lat_rad.sin(),
            lat_rad.cos() * long_rad.cos(),
            id,
        ));
    }
    let octree = KdTree::build_by_ordered_float(points);
    let mut hash_out = AHashMap::with_capacity(octree.len() * 3);
    for station in octree.iter() {
        hash_out.insert(
            station.3,
            octree
                .within_radius(station, max_dist_m as f64 / EARTH_RADIUS_M)
                .iter()
                .filter(|other_station| other_station.3 != station.3)
                .map(|other_station| {
                    (
                        other_station.3,
                        (station.dist(**other_station) * EARTH_RADIUS_M).round() as i32,
                    )
                })
                .collect::<Vec<(i64, i32)>>(),
        );
    }
    hash_out
}
#[cfg(test)]
fn meter_map_to_latlong(stations_m: &[(i64, i64)]) -> Vec<(i64, i64, i64)> {
    stations_m
        .iter()
        .enumerate()
        .map(|(id, (x, y))| {
            (
                id as i64,
                x * 10i64.pow(14) / 111325,
                y * 10i64.pow(14) / 111325,
            )
        })
        .collect::<Vec<_>>()
}
#[cfg(test)]
#[test]
fn test_walking_paths() {
    #[rustfmt::skip]
        let stations_geo = meter_map_to_latlong(
            &[(5i64, 5i64), (1, 10), (10, 1), (10, 10), (100, 0), (100, 6), (110, 6)],
        );
    let paths = generate_walking_paths(&stations_geo, 10);
    assert_eq!(paths.len(), 7);
    assert_eq!(paths.get(&0).unwrap(), &vec![(2, 6), (3, 7), (1, 6)]);
    assert_eq!(paths.get(&1).unwrap(), &vec![(0, 6), (3, 9)]);
    assert_eq!(paths.get(&2).unwrap(), &vec![(0, 6), (3, 9)]);
    assert_eq!(paths.get(&3).unwrap(), &vec![(2, 9), (0, 7), (1, 9)]);
    assert_eq!(paths.get(&4).unwrap(), &vec![(5, 6)]);
    assert_eq!(paths.get(&5).unwrap(), &vec![(6, 10), (4, 6)]);
    assert_eq!(paths.get(&6).unwrap(), &vec![(5, 10)]);
}
#[cfg(test)]
#[test]
fn test_walking_runtime() {
    use std::time::Instant;
    // Set to 1 million when you want to measure runtime
    const STATION_COUNT: usize = 1000;
    let mut stations_m = Vec::with_capacity(STATION_COUNT);
    #[rustfmt::skip]
        stations_m.resize_with(STATION_COUNT, || {
            (rand::random::<u64>() as i64 % 2000, rand::random::<u64>() as i64 % 2000)
        });
    let stations_geo = meter_map_to_latlong(stations_m.as_slice());
    let now = Instant::now();
    let paths = generate_walking_paths(&stations_geo, 3);
    println!("Duration: {:?}", Instant::now().duration_since(now));
    println!(
        "Average connections per: {}",
        paths.iter().map(|item| item.1.len()).sum::<usize>() as f64 / STATION_COUNT as f64
    );
}
