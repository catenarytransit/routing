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

#[derive(Hash, Clone, Copy, Debug, PartialEq, Eq)]
pub struct StationId(i64);
impl From<i64> for StationId {
    fn from(value: i64) -> Self {
        StationId(value)
    }
}

#[derive(Hash, Clone, Copy, Debug, PartialEq, Eq)]
pub struct RouteId(u32);

impl From<u32> for RouteId {
    fn from(value: u32) -> Self {
        RouteId(value)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Route {
    /// Unique identifier of the route
    id: RouteId,
    /// List of stops with the (station_id, stop_id)
    stops: Vec<(StationId, u32)>,
    /// List of starting times of vehicles.
    vehicles: Vec<u32>,
}

/// Encodes a timetable for for many routes and stations.
#[derive(Clone, PartialEq, Debug)]
pub struct Timetable {
    /// Maps route_id to the route timing struct.
    /// The timing struct has a vector of (station_id,time_offset) and a vector of vehicle start times
    pub routes: AHashMap<RouteId, Route>,
    /// Maps station_id to (route_id, stop_index)
    pub lines_per_station: AHashMap<StationId, Vec<(RouteId, usize)>>,
}

#[derive(Clone, PartialEq, Debug)]
pub struct NameMap {
    /// Map station_id to (latitude, longitude, Chateau, stop_id)
    pub stations: AHashMap<i64, (i64, i64, String, String)>,
    // Map (station_id,vehicle_id) to (Chateau, trip_id)
    pub trips: AHashMap<(i64, u32), (String, String)>,
}

impl Timetable {
    /// Returns the index of a station along a route
    fn get_stop_index(&self, station_id: StationId, route_id: RouteId) -> Option<usize> {
        self.lines_per_station
            .get(&station_id)?
            .iter()
            .find(|(route, _idx)| *route == route_id)
            .map(|(_, idx)| *idx as usize)
    }
    fn get_next_vehicle_id(
        &self,
        station_id: StationId,
        route_id: RouteId,
        departing_time: u32,
    ) -> Option<usize> {
        let stop_id = self.get_stop_index(station_id, route_id)?;
        let Route {
            stops: station_offsets,
            vehicles,
            id: _,
        } = self.routes.get(&route_id)?;

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
    /// List of ((departure_station,departure_time), (arrival_station,arrival_time), trip_id, vehicle_id)
    pub legs: Vec<((StationId, u32), (StationId, u32), RouteId, usize)>,
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
    start_station: StationId,
    start_time: u32,
    dest_station: StationId,
    pref: RaptorPref,
) -> AHashMap<(u32, usize), Journey> {
    let Timetable {
        routes,
        lines_per_station,
    } = timetable;

    // Stores a list of earliest arrivals by round and the origin route and departing station id of the arrival.
    // Maps station_id to (time,route_id,departing_station)
    // Arrival time of u32::MAX implies it's not possible in that round
    let mut earliest_arrival_by_round: AHashMap<StationId, Vec<(u32, RouteId, StationId)>> =
        AHashMap::new();
    // Stores the earliest possible arrival time for each station.
    let mut earliest_arrival: AHashMap<StationId, u32> = AHashMap::new();
    earliest_arrival.insert(start_station, start_time);
    earliest_arrival_by_round.insert(start_station, vec![(start_time, RouteId(0), StationId(0))]);
    let mut marked_stops: AHashSet<StationId> = AHashSet::new();
    marked_stops.insert(start_station);
    for k in 1..pref.max_transfer_limit {
        // Stores a list of route ids mapped to (stop_id, stop_index).
        let mut queue: AHashMap<RouteId, (StationId, usize)> = AHashMap::new();
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
            let Route {
                stops: route_stops,
                vehicles: route_vehicles,
                id: _,
            } = routes.get(&route).unwrap();
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
                            occ.get_mut()
                                .resize(k + 1, (u32::MAX, RouteId(0), StationId(0)));
                            occ.get_mut()[k] = (quickest_time, route, departing_station);
                        }
                        Entry::Vacant(ent) => {
                            let mut vec = vec![];
                            vec.resize(k + 1, (u32::MAX, RouteId(0), StationId(0)));
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
        let mut legs: Vec<((StationId, u32), (StationId, u32), RouteId, usize)> =
            Vec::with_capacity(final_round);

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
            let departing_time = routes.get(&route_id).unwrap().vehicles[vehicle_id]
                + routes.get(&route_id).unwrap().stops[departing_stop_id as usize].1;

            legs.push((
                (departing_station, departing_time),
                (cur_station, time),
                route_id,
                vehicle_id,
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

    use crate::raptor::route::Route;
    use crate::raptor::route::RouteId;
    use crate::raptor::route::StationId;

    use super::Journey;
    use super::RaptorPref;
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
        let r_1 = RouteId(1);
        let r_2 = RouteId(2);
        let r_3 = RouteId(3);
        let r_4 = RouteId(4);
        let r_5 = RouteId(5);
        #[rustfmt::skip]
        let timetable = Timetable {
            routes: AHashMap::from([
                (r_1, Route { 
                    id: r_1,
                    stops: vec![(StationId(0), 0), (StationId(1), 10), (StationId(2), 15)], 
                    vehicles: vec![0, 5]
                }),
                (r_2, Route {
                    id: r_2,
                    stops: vec![(StationId(0), 0), (StationId(3), 10), (StationId(6), 20), (StationId(13), 30), (StationId(14), 40)], 
                    vehicles: vec![5, 15],
                }),
                (r_3, Route {
                    id: r_3,
                    stops: vec![(StationId(0), 0), (StationId(1), 5), (StationId(4), 15), (StationId(7), 20), (StationId(8), 30), (StationId(10), 60)], 
                    vehicles: vec![0, 5, 20]
                }),
                (r_4, Route {
                    id: r_3,
                    stops: vec![(StationId(2), 0), (StationId(5), 5), (StationId(9), 10), (StationId(10), 15)], 
                    vehicles: vec![0, 5, 10, 15, 20, 25],
                }),
                (r_5, Route {
                    id: r_3,
                    stops: vec![(StationId(13), 0), (StationId(12), 10), (StationId(11), 20), (StationId(10), 30)], 
                    vehicles: vec![0, 10, 12, 20, 25, 30],
                }),
            ]),
            lines_per_station: AHashMap::from([
                (StationId(0), vec![(r_1, 0), (r_2, 0), (r_3, 0)]),
                (StationId(1), vec![(r_1, 1), (r_3, 1)]),
                (StationId(2), vec![(r_1, 2), (r_4, 0)]),
                (StationId(3), vec![(r_2, 1)]),
                (StationId(4), vec![(r_3, 2)]),
                (StationId(5), vec![(r_4, 1)]),
                (StationId(6), vec![(r_2, 2)]),
                (StationId(7), vec![(r_3, 3)]),
                (StationId(8), vec![(r_3, 4)]),
                (StationId(9), vec![(r_4, 2)]),
                (StationId(10), vec![(r_4, 3), (r_3, 5), (r_5, 3)]),
                (StationId(11), vec![(r_5, 2)]),
                (StationId(12), vec![(r_5, 1)]),
                (StationId(13), vec![(r_5, 0), (r_2, 3)]),
                (StationId(14), vec![(r_2, 4)]),
            ]),
        };
        let output = route_raptor(
            &timetable,
            StationId(0),
            0,
            StationId(10),
            RaptorPref {
                max_transfer_limit: 5,
                minimum_layover_s: 3,
                maximum_footpath_len_m: 0,
            },
        );

        // Direct but slow route (60 seconds)
        assert_eq!(
            output.get(&(60, 1)),
            Some(&Journey {
                legs: vec![((StationId(0), 5), (StationId(10), 65), RouteId(3), 1)]
            })
        );
        // Follow route 1 then route 4 in 35 seconds.
        assert_eq!(
            output.get(&(35, 2)),
            Some(&Journey {
                legs: vec![
                    // Go from station 0 to station 2 on route 1 with vehicle 0
                    ((StationId(0), 5), (StationId(2), 20), RouteId(1), 1),
                    // Go from station 2 to station 10 on route 4 with vehicle 3. Wait 5 seconds for layover because I set minimum layover time to 3 seconds, missing the train that was at the exact same second.
                    ((StationId(2), 25), (StationId(10), 40), RouteId(4), 5)
                ]
            }),
        );
        // Assert there are only two optimum routes.
        assert_eq!(output.len(), 2);
    }
}
