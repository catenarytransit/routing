pub mod road_network;

pub use crate::road_network::road_network::RoadNetwork;

#[allow(unused)]
pub mod road_dijkstras {
    use crate::road_network;
    //routing algorithms and helper functiions
    use crate::road_network::*;
    use crate::RoadNetwork;
    use rand::Rng;
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashMap, HashSet};
    use std::hash::Hash;
    use std::path;
    use std::rc::Rc;
    use std::time::Instant;
    
    use crate::road_network::road_network::Node;

    pub struct RoadDijkstra {
        //handle dijkstra calculations
        pub graph: RoadNetwork,
        pub visited_nodes: HashMap<i64, u64>,
        cost_upper_bound: u64,
        max_settled_nodes: u64,
    }

    #[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord, Hash)]
    pub struct RoadPathedNode {
        //node that references parent nodes, used to create path from goal node to start node
        pub node_self: Node,
        pub distance_from_start: u64,
        pub parent_node: Option<Rc<RoadPathedNode>>,
    }

    impl RoadPathedNode {
        pub fn extract_parent<RoadPathedNode: std::clone::Clone>(
            //returns parent of a pathed node
            last_elem: Rc<RoadPathedNode>,
        ) -> RoadPathedNode {
            let inner: RoadPathedNode = Rc::unwrap_or_clone(last_elem);
            inner
        }

        pub fn get_path(self) -> (Vec<Node>, u64) {
            //uses reference to find the source node with parent_node == None
            let mut shortest_path: Vec<Node> = Vec::new();
            let mut total_distance: u64 = self.distance_from_start;
            let mut current = self;
            while let Some(previous_node) = current.parent_node {
                shortest_path.push(current.node_self);
                current = RoadPathedNode::extract_parent(previous_node);
            }
            shortest_path.push(current.node_self);
            (shortest_path, total_distance)
        }
    }

    pub fn a_star_heuristic(graph: &RoadNetwork, target: i64) -> HashMap<i64, u64> {
        let tail = *graph.nodes.get(&target).unwrap();
        //for each current i64 id, enter euciladan distance from current to target, divided by max speed on that path
        let heuristics = graph
            .nodes
            .iter()
            .map(|(id, head)| {
                (
                    *id,
                    ((i128::pow(((head.lat - tail.lat) * 111229).into(), 2) as f64
                        / f64::powi(10.0, 14)
                        + i128::pow(((head.lon - tail.lon) * 71695).into(), 2) as f64
                            / f64::powi(10.0, 14))
                    .sqrt() as u64)
                        / ((110_f64) * 5.0 / 18.0) as u64, //110 is motorway speed --> max speed possible on road network
                )
            })
            .collect::<HashMap<i64, u64>>();
        heuristics
    }

    impl RoadDijkstra {
        //implementation of dijkstra's shortest path algorithm
        pub fn new(graph: &RoadNetwork) -> Self {
            let visited_nodes = HashMap::new();
            Self {
                graph: graph.clone(),
                visited_nodes,
                cost_upper_bound: u64::MAX,
                max_settled_nodes: u64::MAX,
            }
        }

        pub fn set_cost_upper_bound(&mut self, upper_bound: u64) {
            self.cost_upper_bound = upper_bound;
        }

        pub fn set_max_settled_nodes(&mut self, max_settled: u64) {
            self.max_settled_nodes = max_settled;
        }

        pub fn get_neighbors(
            &mut self,
            current: &RoadPathedNode,
            consider_arc_flags: bool,
        ) -> Vec<(Node, u64)> {
            //return node id of neighbors
            let mut paths = Vec::new();
            let mut next_node_edges = HashMap::new();
            //need some case to handle neighbor to parent instead of just parent to neighbor
            if let Some(connections) = self.graph.edges.get_mut(&current.node_self.id) {
                next_node_edges.clone_from(connections);
            }
            for path in next_node_edges {
                if self.visited_nodes.contains_key(&path.0) {
                    continue;
                }
                if (consider_arc_flags && !path.1 .1) {
                    continue;
                }
                paths.push((*self.graph.nodes.get(&path.0).unwrap(), path.1 .0));
            }
            paths
        }

        pub fn dijkstra(
            &mut self,
            source_id: i64,
            target_id: i64,
            heuristics: &Option<HashMap<i64, u64>>,
            consider_arc_flags: bool,
        ) -> (Option<RoadPathedNode>, HashMap<i64, i64>) {
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, RoadPathedNode)>> = BinaryHeap::new();
            let mut previous_nodes = HashMap::new();

            //set target (-1) for all-node-settle rather than just target settle or smth
            self.visited_nodes.clear();

            let source = *self
                .graph
                .nodes
                .get(&source_id)
                .unwrap_or_else(|| panic!("source node not found"));

            let source_node: RoadPathedNode = RoadPathedNode {
                node_self: (source),
                distance_from_start: 0,
                parent_node: (None),
            };

            //stores distances of node relative to target
            let mut gscore: HashMap<i64, u64> = HashMap::new();
            gscore.insert(source_id, 0);

            priority_queue.push(Reverse((0, source_node.clone())));

            let mut target: Node = Node {
                id: 0,
                lon: 0,
                lat: 0,
            };
            if target_id > 0 {
                target = *self
                    .graph
                    .nodes
                    .get(&target_id)
                    .unwrap_or_else(|| panic!("target node not found"));
            }

            let mut counter = 1;
            let mut cost = 0;
            while !priority_queue.is_empty() {
                let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
                cost = pathed_current_node.distance_from_start;
                let idx = pathed_current_node.node_self.id;

                self.visited_nodes.insert(idx, cost);

                //found target node
                if idx.eq(&target_id) {
                    return (Some(pathed_current_node), previous_nodes);
                }

                //stop conditions
                //cost or # of settled nodes goes over limit
                if cost > self.cost_upper_bound
                    || self.visited_nodes.len() > self.max_settled_nodes as usize
                {
                    return (None, previous_nodes);
                }

                //cost is higher than current path (not optimal)
                if cost > *gscore.get(&idx).unwrap_or(&u64::MAX) {
                    continue;
                }

                for neighbor in self.get_neighbors(&pathed_current_node, consider_arc_flags) {
                    let temp_distance = pathed_current_node.distance_from_start + neighbor.1;
                    let next_distance = *gscore.get(&neighbor.0.id).unwrap_or(&u64::MAX);

                    if temp_distance < next_distance {
                        gscore.insert(neighbor.0.id, temp_distance);
                        let prev_node: Rc<RoadPathedNode> = Rc::new(pathed_current_node.clone());
                        let tentative_new_node = RoadPathedNode {
                            node_self: neighbor.0,
                            distance_from_start: temp_distance,
                            parent_node: Some(prev_node),
                        };
                        let h;
                        if let Some(heuristic) = heuristics {
                            h = heuristic.get(&neighbor.0.id).unwrap_or(&0);
                        } else {
                            h = &0;
                        }
                        //fscore = temp_distance (gscore) + h (hscore)
                        priority_queue.push(Reverse((temp_distance + h, tentative_new_node)));
                        previous_nodes.insert(neighbor.0.id, pathed_current_node.node_self.id);
                    }
                }
                counter += 1;
            }
            (None, previous_nodes)
        }

        pub fn get_random_node_id(&mut self) -> Option<i64> {
            //returns ID of a random valid node from a graph
            let mut rng = rand::thread_rng();
            let mut full_node_list = &self.graph.raw_nodes;
            let mut random: usize = rng.gen_range(0..full_node_list.len());
            let mut node_id = full_node_list.get(random).unwrap();

            Some(*node_id)
        }

        pub fn get_random_node_area_id(
            &mut self,
            lat_min: f32,
            lat_max: f32,
            lon_min: f32,
            lon_max: f32,
        ) -> i64 {
            let lat_range =
                (lat_min * f32::powi(10.0, 7)) as i64..(lat_max * f32::powi(10.0, 7)) as i64;
            let lon_range =
                (lon_min * f32::powi(10.0, 7)) as i64..(lon_max * f32::powi(10.0, 7)) as i64;
            let mut found = false;
            let mut id = -1;
            while (!found) {
                if let Some(node_id) = self.get_random_node_id() {
                    if let Some(node) = self.graph.nodes.get(&node_id) {
                        found = lat_range.contains(&node.lat) && lon_range.contains(&node.lon);
                        id = node_id
                    }
                }
            }
            id
        }

        pub fn get_unvisted_node_id(
            //returns the first unvisted node that function parses upon (used to find largest connected component)
            &mut self,
            other_located_nodes: &HashMap<i64, i32>,
        ) -> Option<i64> {
            if other_located_nodes.len() == self.graph.nodes.len() {
                println!("all nodes visted");
                return None;
            }
            let other_located_nodes = other_located_nodes
                .iter()
                .filter(|(id, count)| **count > 0)
                .map(|(id, _)| id)
                .collect::<Vec<&i64>>();

            for node in &self.graph.nodes {
                if !other_located_nodes.contains(&node.0) {
                    return Some(*node.0);
                }
            }
            None
        }

        pub fn reset_all_flags(&mut self, state: bool) {
            for (_, edgelist) in self.graph.edges.iter_mut() {
                for edge in edgelist.iter_mut() {
                    edge.1 .1 = state;
                }
            }
        }
    }
}

pub mod transit_network {
    //constructs and preprocesses the graph struct from OSM data
    use crate::transit_dijkstras::*;
    use gtfs_structures::*;
    use std::{
        collections::{HashMap, HashSet},
        hash::Hash,
    };

    #[derive(Debug, PartialEq, Hash, Eq, Clone, Copy, PartialOrd, Ord)]
    pub struct NodeId {
        //0 = "untyped"    1 = "arrival"   2 = "transfer"  3 = "departure"
        pub node_type: i8,
        pub station_id: i64,
        pub time: Option<u64>,
        pub trip_id: u64,
        pub lat: i64, //f64 * f64::powi(10.0, 14) as i64
        pub lon: i64, //f64 * f64::powi(10.0, 14) as i64
    }

    pub fn read_from_gtfs_zip(path: &str) -> Gtfs {
        let gtfs = gtfs_structures::GtfsReader::default()
            .read_shapes(false) // Won’t read shapes to save time and memory
            .read(path)
            .ok();
        gtfs.unwrap()
    }

    pub fn calendar_date_filter(
        given_weekday: &str,
        service_id: &str,
        calendar: &Calendar,
    ) -> Option<String> {
        let day_is_valid = match given_weekday {
            "monday" => calendar.monday,
            "tuesday" => calendar.tuesday,
            "wednesday" => calendar.wednesday,
            "thursday" => calendar.thursday,
            "friday" => calendar.friday,
            "saturday" => calendar.saturday,
            "sunday" => calendar.saturday,
            _ => false,
        };

        if day_is_valid {
            Some(service_id.to_owned())
        } else {
            None
        }
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct TimeExpandedGraph {
        //graph struct that will be used to route
        pub day_of_week: String,
        pub transfer_buffer: u64,
        pub nodes: HashSet<NodeId>,
        pub edges: HashMap<NodeId, HashMap<NodeId, u64>>, // tail.id, <head.id, cost>
        pub station_mapping: HashMap<String, i64>, //station_id string, station_id (assigned number)
        pub nodes_per_station: HashMap<i64, Vec<(u64, NodeId)>>,
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct LineConnectionTable {
        //node that references parent nodes, used to create path from goal node to start node
        pub route_id: String,
        pub times_from_start: HashMap<i64, (u64, u16)>, //<stationid, (time from start, sequence number)>
        pub start_times: Vec<u64>,                      //start times for vehicle from first station
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct DirectConnections {
        pub route_tables: HashMap<String, LineConnectionTable>, //route_id, table
        pub lines_per_station: HashMap<i64, HashMap<String, u16>>, //<stationid, <routeid, stop sequence#>>
    }

    impl TimeExpandedGraph {
        pub fn new(
            mut gtfs: Gtfs,
            mut day_of_week: String,
            transfer_buffer: u64,
        ) -> (Self, DirectConnections) {
            day_of_week = day_of_week.to_lowercase();
            //init new transit network graph based on results from reading GTFS zip
            let mut nodes: HashSet<NodeId> = HashSet::new(); //maps GTFS stop id string to sequential numeric stop id
            let mut edges: HashMap<NodeId, HashMap<NodeId, u64>> = HashMap::new();
            let mut station_mapping: HashMap<String, i64> = HashMap::new();
            let mut nodes_per_station: HashMap<i64, Vec<(u64, NodeId)>> = HashMap::new(); // <stationid, (time, node_id)>, # of stations and # of times
            let mut connection_table_per_line: HashMap<String, LineConnectionTable> =
                HashMap::new();
            let mut lines_per_station: HashMap<i64, HashMap<String, u16>> = HashMap::new();

            let service_ids_of_given_day: HashSet<String> = gtfs
                .calendar
                .iter()
                .filter_map(|(service_id, calendar)| {
                    calendar_date_filter(day_of_week.as_str(), service_id, calendar)
                })
                .collect();

            let trip_ids_of_given_day: HashSet<String> = gtfs
                .trips
                .iter()
                .filter(|(_, trip)| service_ids_of_given_day.contains(&trip.service_id))
                .map(|(trip_id, _)| trip_id.to_owned())
                .collect();

            //TODO: add repetitions of trip_id for frequencies.txt if it exists

            let mut iterator: i64 = 0;
            for stop_id in gtfs.stops.iter() {
                station_mapping.insert(stop_id.0.clone(), iterator);
                iterator += 1;
            }
            println!("# of stations: {}", iterator);

            let mut trip_id: u64 = 0; //custom counter like with stop_id
            let mut nodes_by_time: Vec<(u64, NodeId)> = Vec::new();

            for (_, trip) in gtfs.trips.iter_mut() {
                if !trip_ids_of_given_day.contains(&trip.id) {
                    continue;
                }
                let mut id;

                let mut prev_departure: Option<(NodeId, u64)> = None;

                trip.stop_times
                    .sort_by(|a, b| a.stop_sequence.cmp(&b.stop_sequence));

                let trip_start_time: u64 = trip
                    .stop_times
                    .first()
                    .unwrap()
                    .arrival_time
                    .unwrap()
                    .into();
                let mut stations_time_from_trip_start = HashMap::new();

                for stoptime in trip.stop_times.iter() {
                    id = *station_mapping.get(&stoptime.stop.id).unwrap();

                    //write a function that traces up parent stations for lat and lon if unwrap fails (optional value)
                    //if let Some(other_stop_id) = stoptime.stop.parent_station {
                    //
                    //} else{
                    let lat = (stoptime.stop.latitude.unwrap() * f64::powi(10.0, 14)) as i64;
                    let lon = (stoptime.stop.longitude.unwrap() * f64::powi(10.0, 14)) as i64;
                    //}

                    let arrival_time: u64 = stoptime.arrival_time.unwrap().into();
                    let departure_time: u64 = stoptime.departure_time.unwrap().into();

                    stations_time_from_trip_start
                        .insert(id, (arrival_time - trip_start_time, stoptime.stop_sequence));

                    let arrival_node = NodeId {
                        node_type: 1,
                        station_id: id,
                        time: Some(arrival_time),
                        trip_id,
                        lat,
                        lon,
                    };
                    let transfer_node = NodeId {
                        node_type: 2,
                        station_id: id,
                        time: Some(arrival_time + transfer_buffer),
                        trip_id,
                        lat,
                        lon,
                    };
                    let departure_node = NodeId {
                        node_type: 3,
                        station_id: id,
                        time: Some(departure_time),
                        trip_id,
                        lat,
                        lon,
                    };

                    nodes.insert(arrival_node);
                    nodes.insert(transfer_node);
                    nodes.insert(departure_node);

                    if let Some((prev_dep, prev_dep_time)) = prev_departure {
                        edges //travelling arc for previous departure to current arrival
                            .entry(prev_dep) //tail
                            .and_modify(|inner| {
                                inner.insert(arrival_node, arrival_time - prev_dep_time);
                                //head
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(arrival_node, arrival_time - prev_dep_time); //head
                                a
                            });
                    }

                    edges //layover arc for current arrival to current departure
                        .entry(arrival_node) //tail
                        .and_modify(|inner| {
                            inner.insert(departure_node, departure_time - arrival_time);
                            //head
                        })
                        .or_insert({
                            let mut a = HashMap::new();
                            a.insert(departure_node, departure_time - arrival_time); //head
                            a
                        });

                    edges //alighting arc (arrival to transfer)
                        .entry(arrival_node) //tail
                        .and_modify(|inner| {
                            inner.insert(transfer_node, transfer_buffer);
                            //head
                        })
                        .or_insert({
                            let mut a = HashMap::new();
                            a.insert(transfer_node, transfer_buffer); //head
                            a
                        });

                    let node_list = vec![
                        (arrival_time, arrival_node),
                        (arrival_time + transfer_buffer, transfer_node),
                        (departure_time, departure_node),
                    ];

                    nodes_by_time.extend(node_list.iter());

                    nodes_per_station
                        .entry(id)
                        .and_modify(|inner| {
                            inner.extend(node_list.iter());
                        })
                        .or_insert(node_list);

                    prev_departure = Some((departure_node, departure_time));
                }

                trip_id += 1;
                let route_id = trip.route_id.clone();

                connection_table_per_line
                    .entry(route_id.clone())
                    .and_modify(|table| {
                        table.route_id = route_id.clone();
                        table.start_times.push(trip_start_time);
                        table
                            .times_from_start
                            .extend(stations_time_from_trip_start.iter());
                    })
                    .or_insert({
                        LineConnectionTable {
                            route_id,
                            start_times: Vec::from([trip_start_time]),
                            times_from_start: stations_time_from_trip_start,
                        }
                    });
            }
            for (station_id, station) in nodes_per_station.iter_mut() {
                station.sort_by(|a, b| a.0.cmp(&b.0));
                let time_chunks = station.chunk_by_mut(|a, b| a.0 == b.0);

                let mut station_nodes_by_time: Vec<(u64, NodeId)> = Vec::new();
                for chunk in time_chunks {
                    chunk.sort_by(|a, b| a.1.node_type.cmp(&b.1.node_type));
                    station_nodes_by_time.append(&mut chunk.to_vec().to_owned())
                }

                for (current_index, node) in station_nodes_by_time.iter().enumerate() {
                    if node.1.node_type == 2 {
                        for index in current_index + 1..station_nodes_by_time.len() {
                            let future_node = station_nodes_by_time.get(index).unwrap();
                            if future_node.1.node_type == 2 {
                                edges //waiting arc (transfer to transfer)
                                    .entry(node.1) //tail
                                    .and_modify(|inner| {
                                        inner.insert(future_node.1, future_node.0 - node.0);
                                        //head
                                    })
                                    .or_insert({
                                        let mut a = HashMap::new();
                                        a.insert(future_node.1, future_node.0 - node.0); //head
                                        a
                                    });
                                break;
                            }

                            if future_node.1.node_type == 3 {
                                edges //boarding arc (transfer to departure)
                                    .entry(node.1) //tail
                                    .and_modify(|inner| {
                                        inner.insert(future_node.1, future_node.0 - node.0);
                                        //head
                                    })
                                    .or_insert({
                                        let mut a = HashMap::new();
                                        a.insert(future_node.1, future_node.0 - node.0); //head
                                        a
                                    });
                            }
                        }
                    }
                }
                for (route_id, line) in connection_table_per_line.iter() {
                    if let Some((_, sequence_number)) = line.times_from_start.get(station_id) {
                        lines_per_station
                            .entry(*station_id)
                            .and_modify(|map| {
                                map.insert(route_id.clone(), *sequence_number);
                            })
                            .or_insert({
                                let mut map = HashMap::new();
                                map.insert(route_id.clone(), *sequence_number);
                                map
                            });
                    }
                }
            }

            (
                Self {
                    day_of_week,
                    transfer_buffer,
                    nodes,
                    edges,
                    station_mapping,
                    nodes_per_station,
                },
                DirectConnections {
                    route_tables: connection_table_per_line,
                    lines_per_station,
                },
            )
        }

        pub fn reduce_to_largest_connected_component(self) -> Self {
            let saved_day = self.day_of_week.clone();
            let saved_tb = self.transfer_buffer;
            //reduces graph to largest connected component through nodes visited with time_expanded_dijkstra
            let mut counter = 0;
            let mut number_times_node_visted: HashMap<NodeId, i32> = HashMap::new();
            let shortest_path_graph = Dijkstra::new(&self);

            while let Some(source_id) =
                shortest_path_graph.get_unvisted_node_id(&number_times_node_visted)
            {
                counter += 1;
                let mut shortest_path_graph = Dijkstra::new(&self);
                shortest_path_graph.time_expanded_dijkstra(Some(source_id), None, None, &None);
                for node in shortest_path_graph.visited_nodes.keys() {
                    number_times_node_visted.insert(*node, counter);
                }
                if number_times_node_visted.len() > (self.nodes.len() / 2) {
                    break;
                }
            }

            let mut new_node_list: Vec<(&NodeId, &i32)> = number_times_node_visted.iter().collect();
            new_node_list.sort_by(|(_, counter1), (_, counter2)| counter1.cmp(counter2));

            let connected_components =
                &mut new_node_list.chunk_by(|(_, counter1), (_, counter2)| counter1 == counter2);

            let mut largest_node_set = Vec::new();
            let mut prev_set_size = 0;

            for node_set in connected_components.by_ref() {
                if node_set.len() > prev_set_size {
                    largest_node_set = node_set.to_vec();
                    prev_set_size = node_set.len();
                }
            }

            let lcc_nodes = largest_node_set
                .iter()
                .map(|(id, _)| (**id))
                .collect::<HashSet<NodeId>>();

            let mut filtered_edges = HashMap::new();

            for (tail, edge) in self.edges {
                let mut inner = HashMap::new();
                for (head, info) in edge {
                    if lcc_nodes.contains(&head) {
                        inner.insert(head, info);
                    }
                }
                if lcc_nodes.contains(&tail) {
                    filtered_edges.insert(tail, inner);
                }
            }

            Self {
                day_of_week: saved_day,
                transfer_buffer: saved_tb,
                nodes: lcc_nodes,
                edges: filtered_edges,
                station_mapping: self.station_mapping,
                nodes_per_station: self.nodes_per_station,
            }
        }
    }

    //For each station: Hashmap<stationid, (line, stop sequence #)>
    //Line struct: line_id, hashmap<station, time_from_start>, hashset<starttime>
    //Given time and connection: find intersection of route of two stations where
    //(station line start) > (station line end)
    //find (first start time) after given time - (hashmap find start)
    //compute arrival time from (first start time) + (hashmap find end)

    pub fn direct_connection_query(
        connections: &DirectConnections,
        start_station: i64,
        end_station: i64,
        time: u64,
    ) -> Option<(u64, u64)> {
        //departure time from start, arrival time to end
        let start = connections.lines_per_station.get(&start_station).unwrap();
        let end = connections.lines_per_station.get(&end_station).unwrap();

        let mut route = "";
        for (s_route, s_seq) in start {
            for (e_route, e_seq) in end {
                if s_route == e_route && s_seq < e_seq {
                    route = s_route;
                    break;
                }
            }
        }

        let table = connections.route_tables.get(route).unwrap();
        let mut start_times = table.start_times.clone();
        let time_to_start = table.times_from_start.get(&start_station).unwrap().0;
        let time_to_end = table.times_from_start.get(&end_station).unwrap().0;
        start_times.sort();
        if let Some(first_valid_start_time) =
            start_times.iter().find(|&&s| s > (time - time_to_start))
        {

            let departure = first_valid_start_time + time_to_start;
            let arrival = first_valid_start_time + time_to_end;
            Some((departure, arrival))
        } else {
            None
        }
    }
}

pub mod transit_dijkstras {
    //transit_dijkstras algorithms and helper functiions
    use crate::transit_network::*;
    use rand::Rng;
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashMap, HashSet};
    use std::hash::Hash;
    use std::rc::Rc;

    #[derive(Debug, PartialEq, Clone)]
    pub struct Dijkstra {
        //handle time_expanded_dijkstra calculations
        pub graph: TimeExpandedGraph,
        pub visited_nodes: HashMap<NodeId, PathedNode>,
        cost_upper_bound: u64,
        inactive_nodes: HashSet<NodeId>,
    }

    #[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord, Hash)]
    pub struct PathedNode {
        //node that references parent nodes, used to create path from goal node to start node
        pub node_self: NodeId,
        pub cost_from_start: u64,
        pub parent_node: Option<Rc<PathedNode>>,
    }

    impl PathedNode {
        pub fn get_path(self) -> (Vec<NodeId>, u64) {
            //path, cost
            //uses reference to find the source node with parent_node == None
            //vec.get(0) = target node
            let mut shortest_path = Vec::new();
            let total_distance: u64 = self.cost_from_start;
            let mut current_path = &Some(Rc::new(self));
            while let Some(current) = current_path {
                shortest_path.push(current.node_self); //current.node_self
                current_path = &current.parent_node; //current = current.parent_node
            }
            //shortest_path.push(current_path.unwrap()));
            (shortest_path, total_distance)
        }
    }

    impl Dijkstra {
        //implementation of time_expanded_dijkstra's shortest path algorithm
        pub fn new(graph: &TimeExpandedGraph) -> Self {
            let visited_nodes = HashMap::new();
            let inactive_nodes = HashSet::new();
            Self {
                graph: graph.clone(),
                visited_nodes,
                cost_upper_bound: u64::MAX,
                inactive_nodes,
            }
        }

        pub fn set_cost_upper_bound(&mut self, upper_bound: u64) {
            self.cost_upper_bound = upper_bound;
        }

        pub fn get_neighbors(
            &mut self,
            current: &PathedNode,
            hubs: &Option<HashSet<i64>>,
        ) -> Vec<(NodeId, u64)> {
            //return node id of neighbors
            let mut paths = Vec::new();
            let mut next_node_edges = HashMap::new();
            //need some case to handle neighbor to parent instead of just parent to neighbor
            if let Some(connections) = self.graph.edges.get(&current.node_self) {
                next_node_edges.clone_from(connections);
            }
            for (next_node_id, cost) in next_node_edges {
                if self.visited_nodes.contains_key(&next_node_id) {
                    continue;
                }
                if let Some(hub_list) = hubs {
                    if next_node_id.node_type == 2 && hub_list.contains(&next_node_id.station_id) {
                        self.inactive_nodes.insert(current.node_self);
                    }
                }

                paths.push((next_node_id, cost));
            }
            paths
        }

        pub fn time_expanded_dijkstra(
            &mut self,
            source_id: Option<NodeId>,
            source_id_set: Option<Vec<NodeId>>,
            target_id: Option<NodeId>, //if target == None, settles all reachable nodes
            hubs: &Option<HashSet<i64>>,
        ) -> Option<PathedNode> {
            //returns path from the source to target if exists, also path from every node to source
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();

            //stores distances of node relative to target
            let mut gscore: HashMap<NodeId, u64> = HashMap::new();

            self.visited_nodes.clear();

            if let Some(source_id) = source_id {
                let source_node: PathedNode = PathedNode {
                    node_self: (source_id),
                    cost_from_start: 0,
                    parent_node: (None),
                };

                gscore.insert(source_id, 0);

                priority_queue.push(Reverse((0, source_node)));
            } else if let Some(source_id_set) = source_id_set {
                for source_id in source_id_set {
                    let source_node: PathedNode = PathedNode {
                        node_self: source_id,
                        cost_from_start: 0,
                        parent_node: None
                    };

                    gscore.insert(source_id, 0);
                    priority_queue.push(Reverse((0, source_node)));
                }
            }

            let mut current_cost;

            while !priority_queue.is_empty() {
                let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
                current_cost = pathed_current_node.cost_from_start;
                let idx = pathed_current_node.node_self;

                self.visited_nodes.insert(idx, pathed_current_node.clone());

                //found target node
                if let Some(target_id) = target_id {
                    if idx.eq(&target_id) {
                        return Some(pathed_current_node);
                    }
                }

                //stop search for local TP if all unsettled NodeIds are inactive --> all unvisited nodes is subset of inactive nodes
                if hubs.is_some() {
                    let visted_as_set = &self
                        .visited_nodes
                        .keys()
                        .copied()
                        .collect::<HashSet<NodeId>>();
                    let unsettled_nodes: HashSet<_> =
                        self.graph.nodes.difference(visted_as_set).collect();
                    if unsettled_nodes.is_subset(&self.inactive_nodes.iter().collect()) {
                        return None;
                    }
                }

                //stop conditions
                //cost or # of settled nodes goes over limit
                if current_cost > self.cost_upper_bound {
                    return None;
                }

                //cost is higher than current path (not optimal)
                if current_cost > *gscore.get(&idx).unwrap_or(&u64::MAX) {
                    continue;
                }

                for neighbor in self.get_neighbors(&pathed_current_node, hubs) {
                    let temp_distance = current_cost + neighbor.1;
                    let next_distance = *gscore.get(&neighbor.0).unwrap_or(&u64::MAX);

                    if temp_distance < next_distance {
                        gscore.insert(neighbor.0, temp_distance);
                        let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node.clone());
                        let tentative_new_node = PathedNode {
                            node_self: neighbor.0,
                            cost_from_start: temp_distance,
                            parent_node: Some(prev_node),
                        };

                        priority_queue.push(Reverse((temp_distance, tentative_new_node)));
                    }
                }
            }
            None //(None, node_path_tracker)
        }

        pub fn get_random_node_id(&self) -> Option<NodeId> {
            //returns ID of a random valid node from a graph
            let full_node_list = self.graph.nodes.iter().copied().collect::<Vec<NodeId>>();
            let mut rng = rand::thread_rng();
            let random: usize = rng.gen_range(0..full_node_list.len());
            full_node_list.get(random).copied()
        }

        pub fn get_random_start(&self) -> Option<NodeId> {
            //returns ID of a random valid node from a graph
            let full_node_list: Vec<_> = self
                .graph
                .nodes
                .iter()
                .filter(|id| id.node_type == 3 && id.time > Some(21600) && id.time < Some(75600))
                .copied()
                .collect();
            let mut rng = rand::thread_rng();
            let random: usize = rng.gen_range(0..full_node_list.len());
            full_node_list.get(random).copied()
        }

        /*pub fn get_random_end(&self, start_time: u64) -> Option<NodeId> {
            //returns ID of a random valid node from a graph
            let full_node_list: Vec<_> = self
                .graph
                .nodes
                .iter()
                .filter(|id| {
                    id.node_type == 3 && id.time > Some(start_time) && id.time < Some(75600)
                })
                .copied()
                .collect();
            let mut rng = rand::thread_rng();
            let mut random: usize = rng.gen_range(0..full_node_list.len());
            full_node_list.get(random).copied()
        }*/

        //returns the first unvisted node that function parses upon (used to find largest connected component)
        pub fn get_unvisted_node_id(&self, found_nodes: &HashMap<NodeId, i32>) -> Option<NodeId> {
            if found_nodes.len() == self.graph.nodes.len() {
                print!("all nodes visted\t");
                return None;
            }
            let found_nodes = found_nodes
                .iter()
                .filter(|(_, count)| **count > 0)
                .map(|(id, _)| id)
                .collect::<Vec<&NodeId>>();

            for node in &self.graph.nodes {
                if !found_nodes.contains(&node) {
                    return Some(*node);
                }
            }
            None
        }
    }
}

pub mod transfer_patterns {
    //THE FINAL BOSS
    use crate::{road_dijkstras::*, road_network::*, transit_dijkstras::*, transit_network::*};
    use geo::algorithm::haversine_distance::*;
    use geo::point;
    use geo::Point;
    use std::cmp::Reverse;
    use std::collections::hash_map::Entry;
    use std::collections::{BinaryHeap, HashMap, HashSet};
    use std::rc::Rc;
    use std::time::Instant;

    #[derive(Debug, PartialEq, Clone)]
    pub struct TDDijkstra {
        //handle time_expanded_dijkstra calculations
        pub connections: DirectConnections,
        pub edges: HashMap<NodeId, Vec<NodeId>>,
        pub visited_nodes: HashMap<NodeId, PathedNode>,
    }

    impl TDDijkstra {
        //implementation of time_expanded_dijkstra's shortest path algorithm
        pub fn new(connections: DirectConnections, edges: HashMap<NodeId, Vec<NodeId>>) -> Self {
            let visited_nodes = HashMap::new();
            Self {
                connections,
                edges,
                visited_nodes,
            }
        }

        pub fn get_neighbors(
            &self,
            current: &PathedNode,
            connections: &DirectConnections,
        ) -> Vec<(NodeId, u64)> {
            //return node id of neighbors
            let mut paths = Vec::new();
            let mut next_node_edges = HashMap::new();
            //need some case to handle neighbor to parent instead of just parent to neighbor
            if let Some(arcs) = self.edges.get(&current.node_self) {
                for next_node in arcs {
                    if let Some((dept, arr)) = direct_connection_query(
                        connections,
                        current.node_self.station_id,
                        next_node.station_id,
                        current.node_self.time.unwrap(),
                    ) {
                        let cost = arr - dept;
                        next_node_edges.insert(next_node, cost);
                    }
                }
            }
            for (next_node_id, cost) in next_node_edges {
                if self.visited_nodes.contains_key(next_node_id) {
                    continue;
                }

                paths.push((*next_node_id, cost));
            }
            paths
        }

        pub fn time_expanded_dijkstra(
            &mut self,
            source_id: NodeId,
            target_id: NodeId, //if target == None, settles all reachable nodes
        ) -> Option<PathedNode> {
            //returns path from the source to target if exists, also path from every node to source
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();

            //stores distances of node relative to target
            let mut gscore: HashMap<NodeId, u64> = HashMap::new();

            self.visited_nodes.clear();

            let source_node: PathedNode = PathedNode {
                node_self: (source_id),
                cost_from_start: 0,
                parent_node: (None),
            };

            gscore.insert(source_id, 0);

            priority_queue.push(Reverse((0, source_node)));

            let mut current_cost;

            while !priority_queue.is_empty() {
                let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
                current_cost = pathed_current_node.cost_from_start;
                let idx = pathed_current_node.node_self;

                self.visited_nodes.insert(idx, pathed_current_node.clone());

                //found target node
                if idx.eq(&target_id) {
                    return Some(pathed_current_node);
                }

                //cost is higher than current path (not optimal)
                if current_cost > *gscore.get(&idx).unwrap_or(&u64::MAX) {
                    continue;
                }

                for neighbor in self.get_neighbors(&pathed_current_node, &self.connections) {
                    let temp_distance = current_cost + neighbor.1;
                    let next_distance = *gscore.get(&neighbor.0).unwrap_or(&u64::MAX);

                    if temp_distance < next_distance {
                        gscore.insert(neighbor.0, temp_distance);
                        let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node.clone());
                        let tentative_new_node = PathedNode {
                            node_self: neighbor.0,
                            cost_from_start: temp_distance,
                            parent_node: Some(prev_node),
                        };

                        /*let mut h = &0;
                        if let Some(heuristic) = heuristics {
                            h = heuristic.get(&neighbor.0).unwrap_or(&0);
                        } */
                        //fscore = temp_distance + h (which is the gscore + hscore)

                        priority_queue.push(Reverse((temp_distance, tentative_new_node.clone())));
                    }
                }
            }
            None //(None, node_path_tracker)
        }
    }

    pub struct TransferPatterns {
        pub hubs: HashSet<i64>,
        pub transfer_patterns: HashMap<(NodeId, NodeId), Vec<NodeId>>,
    }

    use crate::RoadNetwork;

    impl TransferPatterns {
        pub fn new() -> Self {
            let transfer_patterns = HashMap::new();
            let hubs = HashSet::new();
            TransferPatterns {
                hubs,
                transfer_patterns,
            }
        }

        //only calculate global time_expanded_dijkstra from hubs (important stations) to save complexity
        pub fn hub_selection(&mut self, router: &Dijkstra, random_samples: u32, cost_limit: u64) {
            //station ids
            let num_stations = 1.max((router.graph.station_mapping.len() as u32) / 100);
            let mut selected_hubs = HashSet::new();

            let mut time_independent_edges: HashMap<NodeId, HashMap<NodeId, u64>> = HashMap::new();

            let mut time_independent_nodes = HashSet::new();

            for (tail, edge) in router.graph.edges.iter() {
                let ti_tail = NodeId {
                    node_type: 0,
                    station_id: tail.station_id,
                    time: None,
                    trip_id: 0,
                    lat: tail.lat,
                    lon: tail.lon,
                };
                time_independent_nodes.insert(ti_tail);
                for (head, cost) in edge {
                    let ti_head = NodeId {
                        node_type: 0,
                        station_id: head.station_id,
                        time: None,
                        trip_id: 0,
                        lat: head.lat,
                        lon: head.lon,
                    };
                    time_independent_nodes.insert(ti_head);
                    time_independent_edges
                        .entry(ti_tail)
                        .and_modify(|map| {
                            //update graph if found edge (u, v) with smaller cost than existing edge (u, v)
                            if let Some(previous_cost) = map.get(head) {
                                if cost < previous_cost {
                                    map.insert(ti_head, *cost);
                                }
                            } else {
                                map.insert(ti_head, *cost);
                            };
                        })
                        .or_insert({
                            let mut map: HashMap<NodeId, u64> = HashMap::new();
                            map.insert(ti_head, *cost);
                            map
                        });
                }
            }

            let time_independent_graph = TimeExpandedGraph {
                day_of_week: router.graph.day_of_week.clone(),
                transfer_buffer: router.graph.transfer_buffer,
                nodes: time_independent_nodes,
                edges: time_independent_edges,
                station_mapping: router.graph.station_mapping.clone(),
                nodes_per_station: router.graph.nodes_per_station.clone(),
            };

            let mut time_independent_router = Dijkstra::new(&time_independent_graph);
            time_independent_router.set_cost_upper_bound(cost_limit);

            let mut hub_list: HashMap<NodeId, u16> = HashMap::new();

            for _ in 0..random_samples {
                let current_node = time_independent_router.get_random_node_id();
                time_independent_router.time_expanded_dijkstra(current_node, None, None, &None);
                for (node, _) in time_independent_router.visited_nodes.iter() {
                    match hub_list.entry(*node) {
                        Entry::Occupied(mut o) => {
                            let counter = o.get_mut();
                            *counter += 1;
                        }
                        Entry::Vacant(v) => {
                            v.insert(1);
                        }
                    }
                }
            }

            //println!("hub list {:?}", hub_list);

            let mut sorted_hubs: BinaryHeap<(u16, NodeId)> =
                hub_list.into_iter().map(|(n, c)| (c, n)).collect();

            for _ in 0..num_stations {
                let hub = sorted_hubs.pop().unwrap();
                selected_hubs.insert(hub.1.station_id);
            }

            self.hubs = selected_hubs;
        }

        // Precompute transfer patterns from a given station to all other stations.
        // Return the transfer patterns & numbers of ybetween each station pair.
        pub fn num_transfer_patterns_from_source(
            &self,
            source_station_id: i64,
            router: &mut Dijkstra,
            hubs: &Option<HashSet<i64>>,
        ) -> HashMap<(NodeId, NodeId), Vec<NodeId>> {

            let source_transfer_nodes: Option<Vec<NodeId>> = Some(
                router
                    .graph
                    .nodes_per_station
                    .get(&source_station_id)
                    .unwrap()
                    .iter()
                    .filter(|(_, node)| node.node_type == 2)
                    //paper says transfer nodes only, slides say all nodes, i followed paper
                    .map(|(_, node)| *node)
                    .collect(),
            );
            
            //note: multilabel time_expanded_dijkstras are always slower due to label set maintenance
            router.time_expanded_dijkstra(None, source_transfer_nodes, None, hubs);
            let result = self.transfer_patterns_to_target(router);

            result
        }

        // Backtrace all paths from a given station pair wrt to the last Dijkstra
        // computation. For each such path, determine its transfer pattern. Return the
        // set of distinct transfer patterns that occurred.
        pub fn transfer_patterns_to_target(
            &self,
            router: &mut Dijkstra,
        ) -> HashMap<(NodeId, NodeId), Vec<NodeId>> {
            
            let mut transfer_patterns = HashMap::new();


            let mut arrival_nodes: Vec<(NodeId, Vec<NodeId>, u64)> = router
                .visited_nodes
                .iter()
                .filter(|(node, _)| node.node_type == 1)
                .map(|(node, pathed_node)| {
                    let (path, cost) = pathed_node.clone().get_path();
                    (*node, path, cost)
                })
                .collect();

            Self::arrival_loop(&mut arrival_nodes);

            for (target, path, _) in arrival_nodes.iter() {
                let mut transfers = Vec::new();
                transfers.push(*target);
                let mut previous_node: NodeId = *target;
                for &node in path {
                    if previous_node.node_type == 3 && node.node_type == 2 {
                        transfers.push(node);
                    }
                    previous_node = node;
                }

                transfers.reverse();

                transfer_patterns.insert((*transfers.first().unwrap(), *target), transfers);
            }

            transfer_patterns
        }

        // Arrival loop for a given station. That is, for each node from that station,
        // see whether its label can be improved by simply waiting from an earlier
        // node at that station.
        pub fn arrival_loop(arrival_nodes: &mut [(NodeId, Vec<NodeId>, u64)]) {
            arrival_nodes.sort_unstable_by(|a, b| a.0.station_id.cmp(&b.0.station_id));
            let time_chunks = arrival_nodes.chunk_by_mut(|a, b| a.0.station_id <= b.0.station_id);
            for chunk in time_chunks {
                chunk.sort_unstable_by(|a, b| a.0.time.cmp(&b.0.time));
                let mut previous_arrival: Option<(NodeId, u64)> = None;
                for (node, path, cost) in chunk.iter_mut() {
                    if let Some((prev_node, prev_cost)) = previous_arrival {
                        let new_cost = prev_cost + (node.time.unwrap() - prev_node.time.unwrap());
                        if new_cost <= *cost {
                            *cost = new_cost;
                            path.insert(1, prev_node);
                        }
                    }
                    previous_arrival = Some((*node, *cost));
                }
                //new_arrival_list.append(&mut chunk.to_vec().to_owned())
            }
            //new_arrival_list
        }

        pub fn make_points_from_coords(
            source_lat: f64,
            source_lon: f64,
            target_lat: f64,
            target_lon: f64,
        ) -> (Point, Point) {
            let source = point!(x:source_lat, y:source_lon);
            let target = point!(x: target_lat, y: target_lon);
            (source, target)
        }

        pub fn query_graph_construction_from_geodesic_points(
            &mut self,
            router: &mut Dijkstra,
            source: Point,
            target: Point,
            time: u64,
            preset_distance: f64, //in meters
        ) -> (Vec<NodeId>, Vec<NodeId>, HashMap<NodeId, Vec<NodeId>>) {
            //source nodes, target nodes, edges

            //compute sets of N(source) and N(target) of stations N= near
            let sources: Vec<_> = router
                .graph
                .nodes
                .iter()
                .filter(|node| {
                    let node_coord = point!(x: node.lat as f64 / f64::powi(10.0, 14), y: node.lon as f64 / f64::powi(10.0, 14));
                    source.haversine_distance(&node_coord) <= preset_distance
                        && node.time >= Some(time)
                })
                .copied()
                .collect();

            let targets: Vec<_> = router
                .graph
                .nodes
                .iter()
                .filter(|node| {
                    let node_coord = point!(x: node.lat as f64 / f64::powi(10.0, 14), y: node.lon as f64 / f64::powi(10.0, 14));
                    target.haversine_distance(&node_coord) <= preset_distance
                })
                .copied()
                .collect();

            //get hubs of important stations I(hubs)
            self.hub_selection(router, 10000, 54000); //cost limit at 15 hours, arbitrary

            let hubs = Some(self.hubs.clone());
            //precompute local TP from N(source) to first hub (this min hub is access station)
            for source in sources.iter() {
                self.num_transfer_patterns_from_source(source.station_id, router, &hubs);
            }

            //global transfer patterns from I(hubs) to to N(target())
            for hub_station in self.hubs.iter() {
                let tps = self.num_transfer_patterns_from_source(*hub_station, router, &None);
                self.transfer_patterns.extend(tps.into_iter());
            }

            let mut raw_edges = HashMap::new();
            let _ = self
                .transfer_patterns
                .iter()
                .filter(|((source, target), _)| {
                    sources.contains(source) && targets.contains(target)
                })
                .map(|(_, path)| {
                    let mut prev = None;
                    for node in path {
                        if let Some(prev) = prev {
                            match raw_edges.entry(prev) {
                                Entry::Occupied(mut o) => {
                                    let tails: &mut Vec<NodeId> = o.get_mut();
                                    tails.push(*node);
                                }
                                Entry::Vacant(v) => {
                                    let tails = Vec::from([*node]);
                                    v.insert(tails);
                                }
                            }
                        }
                        prev = Some(*node);
                    }
                });

            (sources, targets, raw_edges)
        }

        pub fn query_graph_search(
            roads: RoadNetwork,
            connections: DirectConnections,
            edges: HashMap<NodeId, Vec<NodeId>>,
            start: Point,
            end: Point,
            sources: Vec<NodeId>,
            targets: Vec<NodeId>,
        ) {
            let mut router = TDDijkstra::new(connections, edges);

            let mut source_paths: HashMap<&NodeId, Option<RoadPathedNode>> = HashMap::new();
            //note: remember to write a function that returns the closest approx point from road network if cannot find a point!
            if let Some(start_road_node) = roads.nodes.values().find(|n| {
                n.lat == (start.0.x * f64::powi(10.0, 14)) as i64
                    && n.lon == (start.0.y * f64::powi(10.0, 14)) as i64
            }) {
                for source in sources.iter() {
                    let mut graph = RoadDijkstra::new(&roads);
                    if let Some(station_sought) = roads
                        .nodes
                        .values()
                        .find(|n| n.lat == source.lat && n.lon == source.lon)
                    {
                        let result =
                            graph.dijkstra(start_road_node.id, station_sought.id, &None, true);
                        source_paths.insert(source, result.0);
                    }
                }
            }

            let mut target_paths: HashMap<&NodeId, Option<RoadPathedNode>> = HashMap::new();
            //note: remember to write a function that returns the closest approx point from road network if cannot find a point!
            if let Some(end_road_node) = roads.nodes.values().find(|n| {
                n.lat == (end.0.x * f64::powi(10.0, 14)) as i64
                    && n.lon == (end.0.y * f64::powi(10.0, 14)) as i64
            }) {
                for target in targets.iter() {
                    let mut graph = RoadDijkstra::new(&roads);
                    if let Some(station_sought) = roads
                        .nodes
                        .values()
                        .find(|n| n.lat == target.lat && n.lon == target.lon)
                    {
                        let result =
                            graph.dijkstra(end_road_node.id, station_sought.id, &None, true);
                        target_paths.insert(target, result.0);
                    }
                }
            }

            let mut transit_paths = HashMap::new();
            for source_id in sources.iter() {
                let source_path = source_paths.get(source_id).unwrap().as_ref().unwrap();
                for target_id in targets.iter() {
                    let target_path = target_paths.get(target_id).unwrap().as_ref().unwrap();
                    let path = router.time_expanded_dijkstra(*source_id, *target_id);
                    let transit_cost = path.as_ref().unwrap().cost_from_start;

                    transit_paths.insert(
                        (source_id, target_id),
                        (
                            (source_path, path, target_path),
                            transit_cost
                                + source_path.distance_from_start
                                + target_path.distance_from_start,
                        ),
                    );
                }
            }
        }
    }
}
