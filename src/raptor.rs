use std::collections::hash_map::Entry;

// File written by Benjamin Cates
use ahash::AHashMap;
use ahash::AHashSet;

/// Encodes preferences for a raptor routing query
#[derive(Clone, PartialEq, Debug)]
pub struct RaptorPref {
    pub max_transfer_limit: usize,
    pub minimum_layover_s: u32,
    pub maximum_footpath_len_m: u32,
}

/// Encodes a timetable for a public transit agency.
#[derive(Clone, PartialEq, Debug)]
pub struct Timetable {
    /// Maps route_id to the route timing struct.
    /// The timing struct has a vector of (station_id,time_offset) and a vector of vehicle start times
    pub routes: AHashMap<u32, (Vec<(i64, u32)>, Vec<u32>)>,
    /// Maps station_id to (route_id, stop_index)
    pub lines_per_station: AHashMap<i64, Vec<(u32, u32)>>,
}

mod walking {
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
}
pub use walking::generate_walking_paths;

impl Timetable {
    /// Returns the index of a station along a route
    fn get_stop_index(&self, station_id: i64, route_id: u32) -> Option<usize> {
        self.lines_per_station
            .get(&station_id)?
            .iter()
            .find(|(route, _idx)| *route == route_id)
            .map(|(_, idx)| *idx as usize)
    }
    fn get_next_vehicle_id(
        &self,
        station_id: i64,
        route_id: u32,
        departing_time: u32,
    ) -> Option<usize> {
        println!("st{station_id} on r{route_id} @ {departing_time}");
        let stop_id = self.get_stop_index(station_id, route_id)?;
        let (station_offsets, vehicles) = self.routes.get(&route_id)?;

        let start_time = departing_time.checked_sub(station_offsets[stop_id as usize].1)?;
        vehicles
            .iter()
            .enumerate()
            .find(|(_, start)| **start >= start_time)
            .map(|(id, _)| id)
    }
}

/// Encodes a path through a public transit network.
/// Can be decoded into a human-readable format with proper GTFS data.
/// The total time of the journey is the final arrival time minus the first departure time.
#[derive(Clone, PartialEq, Debug)]
pub struct Journey {
    /// List of ((departure_station,departure_time), (arrival_station,arrival_time), trip_id)
    pub legs: Vec<((i64, u32), (i64, u32), u32, u32)>,
}

/// Based on the direct connection dataset, calculate the round-based routing.
/// Algorithm citation: Delling, D., Pajor, T., & Werneck, R. F. (2015). Round-based public transit routing. Transportation Science, 49(3), 591-604.
/// Original paper: https://www.microsoft.com/en-us/research/wp-content/uploads/2012/01/raptor_alenex.pdf
///
/// Takes in the timetable and a mutable output of paths as arguments.
/// Returns the starting station, ending station, and Path as an output if the route is found.
/// Returns a pareto-optimal map of journeys where the key is (distance, num_transfers). The list of trips can be obtained by iterating over the entire AHashMap.
pub fn route_raptor(
    timetable: &Timetable,
    start_station: i64,
    start_time: u32,
    dest_station: i64,
    pref: RaptorPref,
) -> AHashMap<(u32, usize), Journey> {
    let Timetable {
        routes,
        lines_per_station,
    } = timetable;

    // Stores a list of earliest arrivals by round and the origin route and departing station id of the arrival.
    // Maps station_id to (time,route_id,departing_station)
    // Arrival time of u32::MAX implies it's not possible in that round
    let mut earliest_arrival_by_round: AHashMap<i64, Vec<(u32, u32, i64)>> = AHashMap::new();
    // Stores the earliest possible arrival time for each station.
    let mut earliest_arrival: AHashMap<i64, u32> = AHashMap::new();
    earliest_arrival.insert(start_station, start_time);
    earliest_arrival_by_round.insert(start_station, vec![(start_time, 0, 0)]);
    let mut marked_stops: AHashSet<i64> = AHashSet::new();
    marked_stops.insert(start_station);
    for k in 1..pref.max_transfer_limit {
        // Stores a list of route ids mapped to (stop_id, stop_index).
        let mut queue: AHashMap<u32, (i64, u32)> = AHashMap::new();
        // Finds the earliest stop for each route based on the marked stations.
        for stop in marked_stops.drain() {
            let Some(routes_list) = lines_per_station.get(&stop) else {
                continue;
            };
            for (route_id, stop_idx) in routes_list {
                match queue.entry(*route_id) {
                    Entry::Occupied(mut occ) => {
                        // Choose the one with the earliest stop idx since we iterate from that stop idx to the end of the route anyway.
                        let (_cur_route_id, cur_stop_idx) = *occ.get();
                        if cur_stop_idx > *stop_idx {
                            *occ.get_mut() = (stop, *stop_idx);
                        }
                    }
                    Entry::Vacant(vac) => {
                        vac.insert((stop, *stop_idx));
                    }
                }
            }
        }
        println!("Queue: {queue:?}");
        // Iterate through list of marked routes to minimize arrival time for later stations on the route
        for (route, (departing_station, departing_idx)) in queue {
            // Keep track of the earliest trip we can board on the route as we iterate through stations. This is an index into the routes vehicle departure list.
            let mut earliest_trip = usize::MAX;
            let (route_stops, route_vehicles) = routes.get(&route).unwrap();
            // Iterate through the stops later in the route. Update earliest_trip if there is a faster way to get to this route.
            for next_stop in route_stops[(departing_idx as usize)..].iter() {
                // Earliest arrival of this trip to this station
                let quickest_time = route_vehicles
                    .get(earliest_trip)
                    .unwrap_or(&u32::MAX)
                    .saturating_add(next_stop.1);
                // If the earliest arrival is faster than the one already stored, add a back pointer to the earliest arrival matrix.
                if earliest_trip != usize::MAX
                    && quickest_time
                        < u32::min(
                            *earliest_arrival.get(&next_stop.0).unwrap_or(&u32::MAX),
                            *earliest_arrival.get(&dest_station).unwrap_or(&u32::MAX),
                        )
                {
                    earliest_arrival.insert(next_stop.0, quickest_time);
                    // Add (quickest_time,route_id,departing_station) to the kth (round_id) entry of the earliest arrival matrix.
                    match earliest_arrival_by_round.entry(next_stop.0) {
                        Entry::Occupied(mut occ) => {
                            occ.get_mut().resize(k + 1, (u32::MAX, 0, 0));
                            occ.get_mut()[k] = (quickest_time, route, departing_station);
                        }
                        Entry::Vacant(ent) => {
                            let mut vec = vec![];
                            vec.resize(k + 1, (u32::MAX, 0, 0));
                            vec[k] = (quickest_time, route, departing_station);
                            ent.insert(vec);
                        }
                    }
                    marked_stops.insert(next_stop.0);
                }
                // Check if there's an earlier trip we could take and save it for the next stations we check.
                if let Some(rounds) = earliest_arrival_by_round.get(&next_stop.0) {
                    if rounds.len() >= k && rounds[k - 1].0 <= quickest_time {
                        if let Some(id) = timetable.get_next_vehicle_id(
                            next_stop.0,
                            route,
                            rounds[k - 1].0 + pref.minimum_layover_s,
                        ) {
                            earliest_trip = id;
                        }
                    }
                }
            }
        }
        // Handle footpaths
        //TODO: handle footpaths

        // Early stopping
        if marked_stops.len() == 0 {
            break;
        }
    }

    // Collect optimal routes
    let Some(arrivals_by_round) = earliest_arrival_by_round.get(&dest_station) else {
        println!("Never reaches station");
        println!("{earliest_arrival_by_round:?}");
        return AHashMap::new();
    };
    let mut pareto_optimal_paths: AHashMap<(u32, usize), Journey> =
        AHashMap::with_capacity(pref.max_transfer_limit);
    println!("{earliest_arrival_by_round:?}");
    for final_round in 0..(pref.max_transfer_limit.min(arrivals_by_round.len())) {
        if arrivals_by_round[final_round].0 == u32::MAX {
            println!("No route in round {final_round}");
            continue;
        }
        let mut legs: Vec<((i64, u32), (i64, u32), u32, u32)> = Vec::with_capacity(final_round);

        // Follow arrival and departure chain
        let mut cur_station = dest_station;
        for round in (1..=final_round).rev() {
            let (time, route_id, departing_station) =
                earliest_arrival_by_round.get(&cur_station).unwrap()[round];
            let vehicle_id = timetable
                .get_next_vehicle_id(cur_station, route_id, time)
                .unwrap();
            let departing_stop_id = timetable
                .get_stop_index(departing_station, route_id)
                .unwrap();
            let departing_time = routes.get(&route_id).unwrap().1[vehicle_id]
                + routes.get(&route_id).unwrap().0[departing_stop_id as usize].1;

            legs.push((
                (departing_station, departing_time),
                (cur_station, time),
                route_id,
                vehicle_id as u32,
            ));
            cur_station = departing_station;
        }
        legs.reverse();

        if legs.len() == 0 || legs[0].0.0 != start_station {
            panic!("Weird issue: {legs:?}");
        }

        println!("{legs:?}");
        let duration = legs.last().unwrap().1.1 - legs.first().unwrap().0.1;

        pareto_optimal_paths.insert((duration, legs.len()), Journey { legs });
    }
    pareto_optimal_paths
}

#[cfg(test)]
mod tests {
    use ahash::AHashMap;

    use crate::raptor::RaptorPref;

    use super::Journey;
    use super::Timetable;
    use super::route_raptor;

    #[test]
    fn test_raptor_catenaryville() {
        // Fictional town named catenaryville
        // Goal will be getting from station 0 (SE) to station 10 (NW) in pareto-optimal to (duration, legs)
        // Route 1 goes from SE corner to SW corner (0-1-2)
        // Route 2 goes from SW corner to NW corner (2-5-9-10)
        // Route 3 goes across diagonal directly from start to end, but is slow (0-1-4-7-8-10)
        // Route 4 goes from SE corner to NE corner (0-3-6-13-14)
        // Route 5 goes from NE corner to NW corner (13-12-11-10)
        // MAP
        //                    14
        // 10     11    12    13
        // 9   8     7        6
        // 5               4  3
        // 2            1     0
        #[rustfmt::skip]
        let timetable = Timetable {
            routes: AHashMap::from([
                (1, (vec![(0, 0), (1, 10), (2, 15)], vec![0, 5])),
                (2, (vec![(0, 0), (3, 10), (6, 20), (13, 30), (14, 40)], vec![5, 15])),
                (3, (vec![(0, 0), (1, 5), (4, 15), (7, 20), (8, 30), (10, 60)], vec![0, 5, 20])), 
                (4, (vec![(2, 0), (5, 5), (9, 10), (10, 15)], vec![0, 5, 10, 15, 20, 25])),
                (5, (vec![(13, 0), (12, 10), (11, 20), (10, 30)], vec![0, 10, 12, 20, 25, 30])),
            ]),
            lines_per_station: AHashMap::from([
                (0, vec![(1, 0), (2, 0), (3, 0)]),
                (1, vec![(1, 1), (3, 1)]),
                (2, vec![(1, 2), (4, 0)]),
                (3, vec![(2, 1)]),
                (4, vec![(3, 2)]),
                (5, vec![(4, 1)]),
                (6, vec![(2, 2)]),
                (7, vec![(3, 3)]),
                (8, vec![(3, 4)]),
                (9, vec![(4, 2)]),
                (10, vec![(4, 3), (3, 5), (5, 3)]),
                (11, vec![(5, 2)]),
                (12, vec![(5, 1)]),
                (13, vec![(5, 0), (2, 3)]),
                (14, vec![(2, 4)]),
            ]),
        };
        let output = route_raptor(
            &timetable,
            0,
            0,
            10,
            RaptorPref {
                max_transfer_limit: 5,
                minimum_layover_s: 3,
                maximum_footpath_len_m: 0,
            },
        );

        println!("{:?}", output);

        // Direct but slow route (60 seconds)
        assert_eq!(
            output.get(&(60, 1)),
            Some(&Journey {
                legs: vec![((0, 5), (10, 65), 3, 1)]
            })
        );
        // Follow route 1 then route 4 in 35 seconds.
        assert_eq!(
            output.get(&(35, 2)),
            Some(&Journey {
                legs: vec![
                    // Go from station 0 to station 2 on route 1 with vehicle 0
                    ((0, 5), (2, 20), 1, 1),
                    // Go from station 2 to station 10 on route 4 with vehicle 3. Wait 5 seconds for layover because I set minimum layover time to 3 seconds, missing the train that was at the exact same second.
                    ((2, 25), (10, 40), 4, 5)
                ]
            }),
        );
        // Assert there are only two optimum routes.
        assert_eq!(output.len(), 2);
    }
}
