#[allow(unused)]
mod graph_construction {
    //constructs and preprocesses the graph struct from OSM data
    use crate::routing::*;
    use core::time;
    use gtfs_structures::*;
    use std::collections::{HashMap, HashSet};

    #[derive(Debug, PartialEq, Hash, Eq, Clone, Copy, PartialOrd, Ord)]
    pub struct Node {
        //nodes from OSM, each with unique ID and coordinate position
        pub id: NodeId, //2b00 for node type + station id
        pub lat: i64,
        pub lon: i64,
    }

    #[derive(Debug, PartialEq, Hash, Eq, Clone, Copy, PartialOrd, Ord)]
    pub struct NodeId {
        //0 = "untyped"    1 = "arrival"   2 = "transfer"  3 = "departure"
        pub node_type: i8,
        pub station_id: u64,
        pub time: u64,
        pub trip_id: u64,
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
        service_id: &String,
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
            Some(service_id.clone())
        } else {
            None
        }
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct TimeExpandedGraph {
        //graph struct that will be used to route
        pub day_of_week: String,
        pub transfer_buffer: u64,
        pub nodes: HashMap<NodeId, Node>, // <(node.id, time), node>
        pub edges: HashMap<NodeId, HashMap<NodeId, (u64, bool)>>, // tail.id, <head.id, (starttime, trip_id, route_name)>
    }

    pub struct LineConnectionTable {
        //node that references parent nodes, used to create path from goal node to start node
        pub route_id: String,
        pub times_from_start: HashMap<u64, (u64, u16)>, //<stationid, (time from start, sequence number)>
        pub start_times: Vec<u64>,                      //start times for vehicle from first station
    }

    pub struct DirectConnections {
        pub route_tables: HashMap<String, LineConnectionTable>, //route_id, table
        pub lines_per_station: HashMap<u64, HashMap<String, u16>>, //<stationid, <routeid, stop sequence#>>
    }

    impl TimeExpandedGraph {
        pub fn new(
            mut gtfs: Gtfs,
            mut day_of_week: String,
            transfer_buffer: u64,
        ) -> (Self, DirectConnections) {
            //, HashMap<String, DirectConnectionTable>) {
            day_of_week = day_of_week.to_lowercase();
            //init new transit network graph based on results from reading GTFS zip
            let mut nodes: HashMap<NodeId, Node> = HashMap::new(); //maps GTFS stop id string to sequential numeric stop id
            let mut edges: HashMap<NodeId, HashMap<NodeId, (u64, bool)>> = HashMap::new();
            let mut node_mapping: HashMap<&String, u64> = HashMap::new();
            let mut routes_mapping: HashMap<&String, u64> = HashMap::new();
            let mut nodes_per_station: HashMap<u64, Vec<(u64, NodeId)>> = HashMap::new(); // <stationid, (time, node_id)>, # of stations and # of times
            let mut connection_table_per_line: HashMap<String, LineConnectionTable> =
                HashMap::new();
            let mut lines_per_station: HashMap<u64, HashMap<String, u16>> = HashMap::new();

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

            let mut iterator: u64 = 0;
            for stop_id in gtfs.stops.iter() {
                node_mapping.insert(stop_id.0, iterator);
                iterator += 1;
            }
            println!("# of stations: {}", iterator);

            let mut trip_id: u64 = 0; //custom counter like with stop_id

            for (_, trip) in gtfs.trips.iter_mut() {
                if !trip_ids_of_given_day.contains(&trip.id) {
                    continue;
                }
                let mut id: u64 = 0;
                let mut nodes_by_time = Vec::new();

                let mut prev_departure: Option<(NodeId, u64)> = None;

                trip.stop_times
                    .sort_by(|a, b| a.stop_sequence.cmp(&b.stop_sequence));

                let trip_start_time: u64 =
                    trip.stop_times.get(0).unwrap().arrival_time.unwrap().into();
                let mut stations_time_from_trip_start = HashMap::new();

                for stoptime in trip.stop_times.iter() {
                    id = *node_mapping.get(&stoptime.stop.id).unwrap();

                    let lat = (stoptime.stop.latitude.unwrap() * f64::powi(10.0, 7)) as i64;
                    let lon = (stoptime.stop.longitude.unwrap() * f64::powi(10.0, 7)) as i64;

                    let arrival_time: u64 = stoptime.arrival_time.unwrap().try_into().unwrap();
                    let departure_time: u64 = stoptime.departure_time.unwrap().try_into().unwrap();

                    stations_time_from_trip_start
                        .insert(id, (arrival_time - trip_start_time, stoptime.stop_sequence));

                    let arrival_node = NodeId {
                        node_type: 1,
                        station_id: id,
                        time: arrival_time,
                        trip_id,
                    }; //2b01
                    let transfer_node = NodeId {
                        node_type: 2,
                        station_id: id,
                        time: arrival_time + transfer_buffer,
                        trip_id,
                    }; //2b10
                    let departure_node = NodeId {
                        node_type: 3,
                        station_id: id,
                        time: departure_time,
                        trip_id,
                    }; //2b11

                    nodes.insert(
                        arrival_node,
                        Node {
                            id: arrival_node,
                            lat,
                            lon,
                        },
                    );
                    nodes.insert(
                        transfer_node,
                        Node {
                            id: transfer_node,
                            lat,
                            lon,
                        },
                    );
                    nodes.insert(
                        departure_node,
                        Node {
                            id: departure_node,
                            lat,
                            lon,
                        },
                    );

                    if let Some((prev_dep, prev_dep_time)) = prev_departure {
                        edges //travelling arc for previous departure to current arrival
                            .entry(prev_dep) //tail
                            .and_modify(|inner| {
                                inner.insert(arrival_node, (arrival_time - prev_dep_time, true));
                                //head
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(arrival_node, (arrival_time - prev_dep_time, true)); //head
                                a
                            });
                    }

                    edges //layover arc for current arrival to current departure
                        .entry(arrival_node) //tail
                        .and_modify(|inner| {
                            inner.insert(departure_node, (departure_time - arrival_time, true));
                            //head
                        })
                        .or_insert({
                            let mut a = HashMap::new();
                            a.insert(departure_node, (departure_time - arrival_time, true)); //head
                            a
                        });

                    edges //alighting arc (arrival to transfer)
                        .entry(arrival_node) //tail
                        .and_modify(|inner| {
                            inner.insert(transfer_node, (transfer_buffer, true));
                            //head
                        })
                        .or_insert({
                            let mut a = HashMap::new();
                            a.insert(transfer_node, (transfer_buffer, true)); //head
                            a
                        });

                    nodes_by_time.push((arrival_time, arrival_node));
                    nodes_by_time.push((arrival_time + transfer_buffer, transfer_node));
                    nodes_by_time.push((departure_time, departure_node));

                    prev_departure = Some((departure_node, departure_time));
                }
                //nodes_by_time.sort(|a, b| a.0.cmp(&b.0));
                nodes_per_station
                    .entry(id) //tail
                    .and_modify(|inner| {
                        inner.append(&mut nodes_by_time);
                        //head
                    })
                    .or_insert({ nodes_by_time });
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
                            route_id: route_id,
                            start_times: Vec::from([trip_start_time]),
                            times_from_start: stations_time_from_trip_start,
                        }
                    });
            }

            for (station_id, mut station) in nodes_per_station {
                station.sort_by(|a, b| a.0.cmp(&b.0));
                let time_chunks = station.chunk_by_mut(|a, b| a.0 == b.0);

                let mut station_nodes_by_time: Vec<(u64, NodeId)> = Vec::new();
                for chunk in time_chunks {
                    chunk.sort_by(|a, b| a.1.node_type.cmp(&b.1.node_type));
                    station_nodes_by_time.append(&mut chunk.to_vec().to_owned())
                }

                let mut prev_transfer_node: Option<(u64, NodeId)> = None;
                let mut current_index: usize = 0;
                for node in station_nodes_by_time.iter() {
                    current_index += 1;
                    if node.1.node_type == 2 {
                        if let Some((prev_tranfer_time, prev_transfer_id)) = prev_transfer_node {
                            edges //waiting arc (arrival to transfer)
                                .entry(prev_transfer_id) //tail
                                .and_modify(|inner| {
                                    inner.insert(node.1, (node.0 - prev_tranfer_time, true));
                                    //head
                                })
                                .or_insert({
                                    let mut a = HashMap::new();
                                    a.insert(node.1, (node.0 - prev_tranfer_time, true)); //head
                                    a
                                });
                        }

                        for index in current_index..station_nodes_by_time.len() {
                            let future_node = station_nodes_by_time.get(index).unwrap();

                            if future_node.1.node_type == 2 {
                                break;
                            }

                            if future_node.1.node_type == 3 {
                                edges //boarding arc (arrival to transfer)
                                    .entry(node.1) //tail
                                    .and_modify(|inner| {
                                        inner.insert(future_node.1, (future_node.0 - node.0, true));
                                        //head
                                    })
                                    .or_insert({
                                        let mut a = HashMap::new();
                                        a.insert(future_node.1, (future_node.0 - node.0, true)); //head
                                        a
                                    });
                            }
                        }
                        prev_transfer_node = Some(*node);
                    }
                }

                for (route_id, line) in connection_table_per_line.iter() {
                    if let Some((time_from_start, sequence_number)) =
                        line.times_from_start.get(&station_id)
                    {
                        lines_per_station
                            .entry(station_id)
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
            //reduces graph to largest connected component through nodes visited with dijkstra
            let mut counter = 0;
            let mut number_times_node_visted: HashMap<NodeId, i32> = HashMap::new();
            let mut shortest_path_graph = Dijkstra::new(&self);
            let mut max_connections = 0;

            while let Some(source_id) =
                shortest_path_graph.get_unvisted_node_id(&number_times_node_visted)
            {
                counter += 1;
                let mut shortest_path_graph = Dijkstra::new(&self);
                shortest_path_graph.dijkstra(source_id, None, &None, false);
                for node in shortest_path_graph.visited_nodes.keys() {
                    number_times_node_visted.insert(*node, counter);
                }
                if number_times_node_visted.len() > (self.nodes.len() / 2) {
                    break;
                }
            }
            let mut new_node_list = Vec::new();
            new_node_list = number_times_node_visted.iter().collect();
            new_node_list.sort_by(|(node1, counter1), (node2, counter2)| counter1.cmp(counter2));

            let connected_components = &mut new_node_list
                .chunk_by(|(node1, counter1), (node2, counter2)| counter1 == counter2);

            let mut largest_node_set = Vec::new();
            let mut prev_set_size = 0;

            while let Some(node_set) = connected_components.next() {
                if node_set.len() > prev_set_size {
                    largest_node_set = node_set.to_vec();
                    prev_set_size = node_set.len();
                }
            }

            let lcc_nodes = largest_node_set
                .iter()
                .map(|(id, _)| (**id, *self.nodes.get(id).unwrap()))
                .collect::<HashMap<NodeId, Node>>();

            let mut filtered_edges = HashMap::new();

            for (tail, edge) in self.edges {
                let mut inner = HashMap::new();
                for (head, info) in edge {
                    if lcc_nodes.contains_key(&head) {
                        inner.insert(head, info);
                    }
                }
                if lcc_nodes.contains_key(&tail) {
                    filtered_edges.insert(tail, inner);
                }
            }

            Self {
                day_of_week: saved_day,
                transfer_buffer: saved_tb,
                nodes: lcc_nodes,
                edges: filtered_edges,
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
        connections: DirectConnections,
        start_station: u64,
        end_station: u64,
        time: u64,
    ) -> Option<(u64, u64)> {
        //departure, arrival times
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
            let arrival = departure + time_to_end;
            return Some((arrival, departure));
        } else {
            None
        }
    }
}

#[allow(unused)]
mod routing {
    //routing algorithms and helper functiions
    use crate::graph_construction::*;
    use rand::Rng;
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashMap, HashSet};
    use std::hash::Hash;
    use std::path;
    use std::rc::Rc;
    use std::time::Instant;

    pub struct Dijkstra {
        //handle dijkstra calculations
        pub graph: TimeExpandedGraph,
        pub visited_nodes: HashMap<NodeId, u64>,
        cost_upper_bound: u64,
        max_settled_nodes: u64,
    }

    #[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord, Hash)]
    pub struct PathedNode {
        //node that references parent nodes, used to create path from goal node to start node
        pub node_self: Node,
        pub distance_from_start: u64,
        pub parent_node: Option<Rc<PathedNode>>,
    }

    impl PathedNode {
        pub fn extract_parent<PathedNode: std::clone::Clone>(
            //returns parent of a pathed node
            last_elem: Rc<PathedNode>,
        ) -> PathedNode {
            let inner: PathedNode = Rc::unwrap_or_clone(last_elem);
            inner
        }

        pub fn get_path(self) -> (Vec<Node>, u64) {
            //uses reference to find the source node with parent_node == None
            let mut shortest_path: Vec<Node> = Vec::new();
            let mut total_distance: u64 = self.distance_from_start;
            let mut current = self;
            while let Some(previous_node) = current.parent_node {
                shortest_path.push(current.node_self);
                current = PathedNode::extract_parent(previous_node);
            }
            shortest_path.push(current.node_self);
            (shortest_path, total_distance)
        }
    }

    pub fn a_star_heuristic(graph: &TimeExpandedGraph, target: NodeId) -> HashMap<NodeId, u64> {
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
            .collect::<HashMap<NodeId, u64>>();
        heuristics
    }

    impl Dijkstra {
        //implementation of dijkstra's shortest path algorithm
        pub fn new(graph: &TimeExpandedGraph) -> Self {
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
            current: &PathedNode,
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
            source_id: NodeId,
            target_id: Option<NodeId>,
            heuristics: &Option<HashMap<NodeId, u64>>,
            consider_arc_flags: bool,
        ) -> (Option<PathedNode>, HashMap<NodeId, NodeId>) {
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();
            let mut previous_nodes = HashMap::new();

            //set target (-1) for all-node-settle rather than just target settle or smth
            self.visited_nodes.clear();

            let source = *self
                .graph
                .nodes
                .get(&source_id)
                .unwrap_or_else(|| panic!("source node not found"));

            let source_node: PathedNode = PathedNode {
                node_self: (source),
                distance_from_start: 0,
                parent_node: (None),
            };

            //stores distances of node relative to target
            let mut gscore: HashMap<NodeId, u64> = HashMap::new();
            gscore.insert(source_id, 0);

            priority_queue.push(Reverse((0, source_node.clone())));

            let mut target: Node = Node {
                id: NodeId {
                    node_type: 0,
                    station_id: 0,
                    time: 0,
                    trip_id: 0,
                },
                lon: 0,
                lat: 0,
            };

            if let Some(target_id) = target_id {
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
                if let Some(target_id) = target_id {
                    if idx.eq(&target_id) {
                        return (Some(pathed_current_node), previous_nodes);
                    }
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
                        let prev_node: Rc<PathedNode> = Rc::new(pathed_current_node.clone());
                        let tentative_new_node = PathedNode {
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

        pub fn get_random_start(&mut self) -> Option<NodeId> {
            //returns ID of a random valid node from a graph
            let mut rng = rand::thread_rng();
            let mut full_node_list: HashSet<_> = self
                .graph
                .nodes
                .iter()
                .filter(|(id, _)| id.node_type == 3 && id.time > 21600 && id.time < 75600)
                .map(|(&id, _)| id)
                .collect();
            let node_id = full_node_list.iter().next().unwrap();

            Some(*node_id)
        }

        pub fn get_random_end(&mut self, start_time: u64) -> Option<NodeId> {
            //returns ID of a random valid node from a graph
            let mut rng = rand::thread_rng();
            let mut full_node_list: HashSet<_> = self
                .graph
                .nodes
                .iter()
                .filter(|(id, _)| id.node_type == 3 && id.time > start_time && id.time < 75600)
                .map(|(&id, _)| id)
                .collect();
            let node_id = full_node_list.iter().next().unwrap();

            Some(*node_id)
        }

        /*pub fn get_random_node_area_id(
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
        }*/

        pub fn get_unvisted_node_id(
            //returns the first unvisted node that function parses upon (used to find largest connected component)
            &mut self,
            other_located_nodes: &HashMap<NodeId, i32>,
        ) -> Option<NodeId> {
            if other_located_nodes.len() == self.graph.nodes.len() {
                println!("all nodes visted");
                return None;
            }
            let other_located_nodes = other_located_nodes
                .iter()
                .filter(|(id, count)| **count > 0)
                .map(|(id, _)| id)
                .collect::<Vec<&NodeId>>();

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

mod transfer_patterns {
    //THE FINAL BOSS
    //use crate::graph_construction::*;
    //use crate::routing::*;
    //use gtfs_structures::*;
    //use std::collections::{HashMap, HashSet};

    // Precompute transfer patterns from a given station to all other stations.
    // Return the number of transfer patterns between each station pair.
    //Array<int> numTransferPatternsFromStation(int sourceStationId);

    // Arrival loop for a given station. That is, for each node from that station,
    // see whether its label can be improved by simply waiting from an earlier
    // node at that station. See explantions in the lecture for how to compute
    // this easily.
    //void arrivalLoop(int stationId);

    // Backtrace all paths from a given station pair wrt to the last Dijkstra
    // computation. For each such path, determine its transfer pattern. Return the
    // set of distinct transfer patterns that occurred.
    //Set<Array<int>> transferPatternsForStationPair(int targetStationId);
}

fn main() {}

#[cfg(test)]
mod tests {
    //use crate::routing::*;
    use crate::graph_construction::*;
    use std::collections::HashMap;
    //use std::time::Instant};

    #[test]
    fn test() {
        //Pareto-set C ordering
        /*fn pareto_recompute(set: &mut Vec<(i32, i32)>, c_p: (i32, i32)) {
            set.sort_by_key(|(x, y)| if y != &0 { 10 * x + (10 / y) } else { 10 * x });
            let mut incomparable = true;
            for c in set.iter() {
                if (c.0 <= c_p.0 && c.1 <= c_p.1) || (c.0 >= c_p.0 && c.1 >= c_p.1) {
                    incomparable = false;
                }

            }
            if incomparable {
                set.push(c_p);
                set.dedup();
                set.sort_by_key(|(x, y)| if y != &0 { 10 * x + (10 / y) } else { 10 * x });
            }
        }

        let mut set = vec![(1, 5), (7, 1), (2, 4), (4, 3)];
        pareto_recompute(&mut set, (6, 2));
        println!("{:?}", set);
        pareto_recompute(&mut set, (1, 1));
        println!("{:?}", set);
        pareto_recompute(&mut set, (8, 0));
        println!("{:?}", set);
        pareto_recompute(&mut set, (8, 0));
        println!("{:?}\n", set);

        let mut set = vec![(9, 1), (8, 1), (2, 7), (3, 3), (6, 4), (1, 9)];
        pareto_recompute(&mut set, (5, 5));
        println!("{:?}", set);
        pareto_recompute(&mut set, (7, 3));
        println!("{:?}", set);
        pareto_recompute(&mut set, (7, 2));
        println!("{:?}", set);
        */

        /*let connections = DirectConnections {
            route_tables: HashMap::from([("L17".to_string(), LineConnectionTable {
                route_id: "L17".to_string(),
                times_from_start: HashMap::from([(154, (0, 0)), (97, (420, 1)), (987, (720, 2)), (111, (1260, 3))]),
                start_times: Vec::from([297000,33300, 36900, 40800, 44400])
            })]),
            lines_per_station: HashMap::from([(97, HashMap::from([
                    ("L8".to_string(), 4), ("L17".to_string(), 2),("L34".to_string(), 5), ("L87".to_string(), 17)])),
                (111, HashMap::from([
                    ("L9".to_string(), 1), ("L13".to_string(), 5),("L17".to_string(), 4), ("L55".to_string(), 16)
                ]))])
        };

        let query = direct_connection_query(connections, 97, 111, 37200);
        println!("{:?}", query);*/
        /*let now = Instant::now();
        let gtfs = read_from_gtfs_zip("hawaii.gtfs.zip"); //manhattan
        let graph = TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10).0;
        let time = now.elapsed().as_secs_f32();

        println!("time {}", time);
        println!("# of nodes: {}", graph.nodes.len());
        println!(
            "# of edges: {}",
            graph
                .edges
                .iter()
                .map(|(_, edges)| edges.len())
                .sum::<usize>()
        );

        //1,831 / 830,100 / 1,371,298     2s     5ms     2h12m35s
        let now = Instant::now();
        let graph = graph.reduce_to_largest_connected_component();
        let mut time = now.elapsed().as_secs_f32();

        println!("time {}", time);
        println!("# of nodes: {}", graph.nodes.len());
        println!(
            "# of edges: {}",
            graph
                .edges
                .iter()
                .map(|(_, edges)| edges.len())
                .sum::<usize>()
        );

        let mut query_time = Vec::new();
        let mut shortest_path_costs = Vec::new();

        let mut routing_graph = Dijkstra::new(&graph);
        for _ in 0..10 {
            let source_id = routing_graph.get_random_start().unwrap();
            let target_id = Some(routing_graph.get_random_end(source_id.time).unwrap());
            let now = Instant::now();
            let result = routing_graph.dijkstra(source_id, target_id, &None, false);
            time = now.elapsed().as_millis() as f32;
            query_time.push(time);

            if let Some(result) = result.0 {
                let cost = result.distance_from_start;
                shortest_path_costs.push(cost);
            }

            //println!("{} cost {}", x, cost);
        }

        println!(
            "average query time in seconds {}",
            query_time.iter().sum::<f32>() / query_time.len() as f32
        );

        println!(
            "average cost hh::mm:ss {}:{}:{} \n",
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 / 3600,
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 % 3600 / 60,
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 % 60,
        );
        */

        /*let mut edges: HashMap<i64, HashMap<i64, (u64, bool)>> = HashMap::new();
        let mut station = vec![
            (755, 1),
            (755, 1),
            (800, 2),
            (800, 2),
            (800, 3),
            (800, 3),
            (802, 2),
            (807, 2),
            (807, 3),
            (812, 3),
            (817, 2)
        ];
        station.sort_by(|a, b| a.0.cmp(&b.0));
        let time_chunks = station.chunk_by_mut(|a, b| a.0 == b.0);

        let mut station_nodes_by_time: Vec<(u64, i64)> = Vec::new();

        for chunk in time_chunks {
            chunk.sort_by(|a, b| (a.1).cmp(&(b.1)));
            station_nodes_by_time.append(&mut chunk.to_vec().to_owned())
        }
        println!("{:?}", station_nodes_by_time);
        let mut prev_transfer_node: Option<(u64, i64)> = None;
        let mut current_index: usize = 0;
        for node in station_nodes_by_time.iter() {
            println!("{:?}", node);
            current_index += 1;
            if node.1 == 2 {
                if let Some((prev_tranfer_time, prev_transfer_id)) = prev_transfer_node {
                    println!("waiting");
                    edges //waiting arc (arrival to transfer)
                        .entry(prev_transfer_id) //tail
                        .and_modify(|inner| {
                            inner.insert(node.1, (node.0 - prev_tranfer_time, true));
                            //head
                        })
                        .or_insert({
                            let mut a = HashMap::new();
                            a.insert(node.1, (node.0 - prev_tranfer_time, true)); //head
                            a
                        });
                }

                prev_transfer_node = Some(*node);
                println!("new tf {:?}", prev_transfer_node);

                for index in current_index..station_nodes_by_time.len() {
                    let node = station_nodes_by_time.get(index).unwrap();
                    println!("l{:?}", node);

                    if node.1 == 2 {
                        break;
                    }

                    if node.1 == 3 {
                        if let Some((prev_tranfer_time, prev_transfer_id)) = prev_transfer_node {
                            println!("boarding");
                            edges //boarding arc (arrival to transfer)
                                .entry(prev_transfer_id) //tail
                                .and_modify(|inner| {
                                    inner.insert(node.1, (node.0 - prev_tranfer_time, true));
                                    //head
                                })
                                .or_insert({
                                    let mut a = HashMap::new();
                                    a.insert(node.1, (node.0 - prev_tranfer_time, true)); //head
                                    a
                                });
                            prev_transfer_node = Some(*node)
                        }
                    }
                }
            }
        }
        println!("{:?}", edges); */
    }
}
