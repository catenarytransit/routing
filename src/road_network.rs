pub mod road_graph_construction {
    //constructs and preprocesses the graph struct from OSM data
    use crate::road_dijkstras::*;
    use core::fmt;
    use core::ops::Range;
    use std::cmp::min;
    use osmpbfreader::objects::OsmObj;
    use std::collections::HashMap;
    use std::collections::HashSet;

    #[derive(
        Debug,
        PartialEq,
        Hash,
        Eq,
        Clone,
        Copy,
        PartialOrd,
        Ord,
        serde::Serialize,
        serde::Deserialize,
    )]
    pub struct Node {
        //nodes from OSM, each with unique ID and coordinate position
        pub id: i64,
        pub lat: i64,
        pub lon: i64,
    }

    #[derive(Debug, PartialEq, Hash, Eq, Clone, serde::Serialize, serde::Deserialize)]
    pub struct Way {
        //ways from OSM, each with unique ID, speed from highway type, and referenced nodes that it connects
        pub id: i64,
        pub speed: u16,
        pub refs: Vec<i64>,
    }

    #[derive(Debug, PartialEq, Clone, serde::Serialize, serde::Deserialize)]
    pub struct RoadNetwork {
        //graph struct that will be used to route
        pub nodes: HashMap<i64, Node>, // <node.id, node>
        pub edges: HashMap<i64, HashMap<i64, (u64, bool)>>, // tail.id, <head.id, (cost, arcflag)>
        pub raw_ways: Vec<Way>,
        pub raw_nodes: Vec<i64>,
    }

    #[derive(Debug, PartialEq, Clone, serde::Serialize, serde::Deserialize)]
    pub struct CoordRange {
        pub min_lat: i64,
        pub max_lat: i64,
        pub min_lon: i64,
        pub max_lon: i64,
    }
    impl fmt::Display for CoordRange {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
            write!(
                f,
                "_{}_{}_{}_{}",
                self.min_lat, self.max_lat, self.min_lon, self.max_lon
            )
        }
    }
    impl CoordRange {
        pub fn from_deci(min_lat: f32, max_lat: f32, min_lon: f32, max_lon: f32) -> Self {
            Self {
                min_lat: (min_lat * f32::powi(10.0, 7)) as i64,
                min_lon: (min_lon * f32::powi(10.0, 7)) as i64,
                max_lat: (max_lat * f32::powi(10.0, 7)) as i64,
                max_lon: (max_lon * f32::powi(10.0, 7)) as i64,
            }
        }
    }

    pub fn speed_calc(highway: &str) -> Option<u16> {
        //for pedestrians
        //picks speed of highway based on given values, in km/h
        match highway {
            "residential" => Some(4),
            "living_street" => Some(4),
            "pedestrian" => Some(4),
            "footway" => Some(4),
            "steps" => Some(4), //toggle this on/off for accecibility?
            "path" => Some(4),
            "corridor" => Some(4),
            "service" => Some(4),
            "motorway" => Some(1),
            "trunk" => Some(1),
            "primary" => Some(1),
            "secondary" => Some(1),
            "tertiary" => Some(1),
            "motorway_link" => Some(1),
            "trunk_link" => Some(1),
            "primary_link" => Some(1),
            "secondary_link" => Some(1),
            _ => Some(1),
        }
    }

    impl RoadNetwork {
        pub fn new(mut nodes: HashMap<i64, Node>, ways: Vec<Way>) -> Self {
            //init new RoadNetwork based on results from reading .pbf file
            let mut edges: HashMap<i64, HashMap<i64, (u64, bool)>> = HashMap::new();
            for way in ways.clone() {
                let mut previous_head_node_now_tail: Option<&Node> = None;
                let mut previous_head_node_index: usize = 0;
                for i in 0..way.refs.len() - 1 {
                    let tail_id = way.refs[i];
                    let tail: Option<&Node> = match previous_head_node_now_tail {
                        Some(previous_head_node_now_tail) => match previous_head_node_index == i {
                            true => Some(previous_head_node_now_tail),
                            false => nodes.get(&tail_id),
                        },
                        None => nodes.get(&tail_id),
                    };

                    let head_id = way.refs[i + 1];
                    let head = nodes.get(&head_id);
                    if let (Some(tail), Some(head)) = (tail, head) {
                        //following math converts lon/lat into distance of segment
                        let a = i128::pow(((head.lat - tail.lat) * 111229).into(), 2) as f64
                            / f64::powi(10.0, 14);
                        let b = i128::pow(((head.lon - tail.lon) * 71695).into(), 2) as f64
                            / f64::powi(10.0, 14);
                        let c = (a + b).sqrt();
                        let cost = (c / ((way.speed as f64) * 5.0 / 18.0)) as u64 * 1000; //miliseconds needed to traverse segment based on road type
                        let flag = false;
                        edges
                            .entry(tail_id)
                            .and_modify(|inner| {
                                inner.insert(head_id, (cost, flag));
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(head_id, (cost, flag));
                                a
                            });
                        edges
                            .entry(head.id)
                            .and_modify(|inner| {
                                inner.insert(tail_id, (cost, flag));
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(tail_id, (cost, flag));
                                a
                            });
                        previous_head_node_now_tail = Some(head);
                        previous_head_node_index = i + 1;
                    }
                }
            }
            let node_to_remove = nodes
                .iter()
                .filter(|(node, _)| !edges.contains_key(node))
                .map(|(x, _)| *x)
                .collect::<Vec<i64>>();
            for node in &node_to_remove {
                nodes.remove(node);
            }

            Self {
                raw_ways: ways,
                edges,
                raw_nodes: nodes.clone().iter().map(|(&id, _)| id).collect(),
                nodes,
            }
        }

        pub fn read_from_osm_file(path: &str) -> (HashMap<i64, Node>, Vec<Way>) {
            //reads osm.pbf file, values are used to make RoadNetwork
            let mut nodes = HashMap::new();
            let mut ways = Vec::new();
            let path_cleaned = std::path::Path::new(&path);
            let r = std::fs::File::open(path_cleaned).unwrap();
            let mut reader = osmpbfreader::OsmPbfReader::new(r);
            for obj in reader.iter().map(Result::unwrap) {
                match obj {
                    OsmObj::Node(e) => {
                        nodes.insert(
                            e.id.0,
                            Node {
                                id: e.id.0,
                                lat: (e.lat() * f64::powi(10.0, 7)) as i64,
                                lon: (e.lon() * f64::powi(10.0, 7)) as i64,
                            },
                        );
                    }
                    OsmObj::Way(e) => {
                        if let Some(road_type) =
                            e.tags.clone().iter().find(|(k, _)| k.eq(&"highway"))
                            && let Some(speed) = speed_calc(road_type.1.as_str())
                        {
                            ways.push(Way {
                                id: e.id.0,
                                speed,
                                refs: e.nodes.into_iter().map(|x| x.0).collect(),
                            });
                        }
                    }
                    _ => {}
                }
            }
            (nodes, ways)
        }

        pub fn reduce_to_largest_connected_component(self) -> Self {
            //reduces graph to largest connected component through nodes visited with dijkstra
            let mut counter = 0;
            let mut number_times_node_visted: HashMap<i64, i32> = HashMap::new();
            let mut shortest_path_graph = RoadDijkstra::new(&self);

            while let Some(source_id) =
                shortest_path_graph.get_unvisted_node_id(&number_times_node_visted)
            {
                counter += 1;
                let mut shortest_path_graph = RoadDijkstra::new(&self);
                shortest_path_graph.dijkstra(source_id, -1, &None, false);
                for node in shortest_path_graph.visited_nodes.keys() {
                    number_times_node_visted.insert(*node, counter);
                }
                if number_times_node_visted.len() > (self.nodes.len() / 2) {
                    break;
                }
            }
            let mut new_node_list = number_times_node_visted.iter().collect::<Vec<_>>();
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
                .map(|(id, _)| (**id, *self.nodes.get(id).unwrap()))
                .collect::<HashMap<i64, Node>>();

            RoadNetwork::new(lcc_nodes, self.raw_ways)
        }

        //min node count per chunk around 28500 or so?
        pub fn chunk_map(&self, min_node_count: usize) -> Vec<CoordRange> {
            let mut coords = Vec::new();

            let node_list = self.nodes.clone().into_values().collect::<Vec<_>>();

            let lat_1_lon_0: bool = false;
            
            Self::recursive_rectangles(&node_list, min_node_count, &mut coords, lat_1_lon_0);
            
            coords
        }

        pub fn recursive_rectangles(node_list: &[Node], min_node_count: usize, coords: &mut Vec<CoordRange>, lat_lon: bool) {

            let node_count = node_list.len() / 2;
            if node_count < min_node_count {
                return 
            }
            let prev_count = node_list.len();

            let mut temp_list;
            if lat_lon {
                temp_list = node_list.clone().sort_by_key(|n| n.lat);
            }
            else {
                temp_list = node_list.clone().sort_by_key(|n|n.lon);
            }

            let slice_a = &node_list[0..node_count];
            let slice_b = &node_list[node_count..prev_count];

            let box_a = CoordRange {
                min_lat: slice_a[0].lat,
                max_lat: slice_a[node_count - 1].lat,
                min_lon: slice_a[0].lon,
                max_lon: slice_a[node_count - 1].lon,
            };
            coords.push(box_a);            
            let box_b = CoordRange {
                min_lat: slice_b[0].lat,
                max_lat: slice_b[node_count - 1].lat,
                min_lon: slice_b[0].lon,
                max_lon: slice_b[node_count - 1].lon,
            };
            coords.push(box_b);

            Self::recursive_rectangles(&slice_a, min_node_count, coords, !lat_lon);
            Self::recursive_rectangles(&slice_b, min_node_count, coords, !lat_lon);            
        }
    }

    pub fn arc_flags_precompute(coords: CoordRange, dijkstra_graph: &mut RoadDijkstra) -> String {
        let lat_range: Range<i64> = coords.min_lat..coords.max_lat;
        let lon_range: Range<i64> = coords.min_lon..coords.max_lon;
        let mut boundary_node = HashSet::new();
        let region_nodes = dijkstra_graph
            .graph
            .nodes
            .iter()
            .filter(|(_, node)| lat_range.contains(&node.lat) && lon_range.contains(&node.lon))
            .map(|(id, _)| *id)
            .collect::<Vec<i64>>();

        println!("number of nodes in region: {}", region_nodes.len());

        for node in region_nodes.clone() {
            if let Some(edge_list) = dijkstra_graph.graph.edges.get_mut(&node) {
                for edge in edge_list.iter_mut() {
                    if region_nodes.contains(edge.0) {
                        edge.1.1 = true;
                        continue;
                    }
                    boundary_node.insert(node);
                }
            }
        }

        println!("boundary nodes: {}", boundary_node.len());

        for node in boundary_node {
            let (_, edges) = dijkstra_graph.dijkstra(node, -1, &None, false);
            for (head, tail) in edges {
                if let Some(edgelist) = dijkstra_graph.graph.edges.get_mut(&head) {
                    for (&id, (_, arcflag)) in edgelist {
                        if id == tail {
                            *arcflag = true;
                            break;
                        }
                    }
                }
            }
        }

        format!("{coords}")
    }
}
pub mod contraction_hierarchies {
    use crate::road_dijkstras::*;
    use std::{
        cmp::Reverse,
        collections::{BinaryHeap, HashMap},
    };

    pub struct ContractedGraph {
        pub ordered_nodes: HashMap<i64, u32>, //id, order number
    }

    impl Default for ContractedGraph {
        fn default() -> Self {
            Self::new()
        }
    }
    impl ContractedGraph {
        pub fn new() -> ContractedGraph {
            ContractedGraph {
                ordered_nodes: HashMap::new(),
            }
        }

        pub fn compute_random_node_ordering(&mut self, graph: &mut RoadDijkstra, length: usize) {
            //self.ordered_nodes_to_contract.insert(0);
            while self.ordered_nodes.len() < length {
                self.ordered_nodes
                    .insert(graph.get_random_node_id().unwrap_or_default(), 0);
            }
            graph.reset_all_flags(true);
        }

        pub fn contract_node(
            &mut self,
            node_id: i64,
            graph: &mut RoadDijkstra,
            compute_edge_diff_only: bool,
        ) -> (u8, i8) {
            //(#shortcuts, edge difference = #shortcuts - arcs incident to node)
            let mut num_shortcuts: u8 = 0;
            let mut edge_diff: i8 = 0;
            let mut costs_of_uv = HashMap::new();
            let mut costs_of_vw = HashMap::new();
            if let Some(edgelist) = graph.graph.edges.get_mut(&node_id) {
                //if Some
                for (w, (cost, flag)) in edgelist {
                    if *flag {
                        costs_of_vw.insert(*w, *cost);
                        *flag = false;
                        edge_diff -= 1;
                    }
                }
            }

            for (&w, &cost) in costs_of_vw.iter() {
                graph
                    .graph
                    .edges
                    .get_mut(&w)
                    .unwrap()
                    .get_mut(&node_id)
                    .unwrap()
                    .1 = false;
                costs_of_uv.insert(w, cost);
            }

            /*for (u, edgelist) in graph.graph.edges.iter_mut() { //for arcs, undirected
                for (v, (cost, flag)) in edgelist.iter_mut() {
                    if *v == node_id {
                        *flag = false;
                        costs_of_uv.insert(*u, *cost);
                    }
                }
            }*/

            graph.set_cost_upper_bound(
                //costs_of_uv.clone().into_values().max().unwrap_or_default() +
                2 * costs_of_vw.clone().into_values().max().unwrap_or_default(),
            );

            let mut temp_shortcuts = HashMap::new();
            for (u, cost_uv) in costs_of_uv.iter() {
                graph.dijkstra(*u, -1, &None, true);
                for (w, cost_vw) in costs_of_vw.iter() {
                    if w == u {
                        continue;
                    }
                    let path_via_uvw = cost_uv + cost_vw;
                    let &dist_w = graph
                        .visited_nodes
                        .get(w)
                        .unwrap_or(temp_shortcuts.get(&(u, w)).unwrap_or(&u64::MAX));
                    if dist_w > path_via_uvw
                        && temp_shortcuts.insert((u, w), path_via_uvw).is_none()
                        && temp_shortcuts.insert((w, u), path_via_uvw).is_none()
                    {
                        edge_diff += 1;
                        num_shortcuts += 1;
                    }
                }
            }
            if compute_edge_diff_only {
                for (&w, &_) in costs_of_vw.iter() {
                    graph
                        .graph
                        .edges
                        .get_mut(&w)
                        .unwrap()
                        .get_mut(&node_id)
                        .unwrap()
                        .1 = true;
                    graph
                        .graph
                        .edges
                        .get_mut(&node_id)
                        .unwrap()
                        .get_mut(&w)
                        .unwrap()
                        .1 = true;
                }
            } else {
                for ((u, w), path_via_uvw) in temp_shortcuts {
                    graph
                        .graph
                        .edges
                        .get_mut(u)
                        .unwrap()
                        .insert(*w, (path_via_uvw, true));
                }
            }

            (num_shortcuts, edge_diff)
        }

        pub fn ch_precompute(&mut self, graph: &mut RoadDijkstra) -> u64 {
            let mut total_shortcuts = 0;
            graph.set_max_settled_nodes(20);
            //step 1: calculate initial e_d order --> PQ with (k: e_d, v: node)
            let mut priority_queue: BinaryHeap<Reverse<(i8, i64)>> = BinaryHeap::new();
            for &n in graph.graph.raw_nodes.clone().iter() {
                let (_, e_d) = self.contract_node(n, graph, true);
                priority_queue.push(Reverse((e_d, n)));
            }

            //step 2: contract all nodes in order of ed, recalculate heuristic --> Lazy
            //Lazy update: If current node does not have smallest ED, pick next, recompute its ED, repeat until find smallest
            //Neighbours update: After each contraction, recompute EDs, but only for the neighbours of the contracted node
            let mut index = 0;
            while !priority_queue.is_empty() {
                let (_, node_id) = priority_queue.pop().unwrap().0;
                //if self.ordered_nodes.contains_key(&node_id) {continue;}
                let (_, e_d) = self.contract_node(node_id, graph, true);

                if !priority_queue.is_empty() {
                    let (next_ed, _) = priority_queue.peek().unwrap().0;
                    if e_d > next_ed {
                        priority_queue.push(Reverse((e_d, node_id)));
                        continue;
                    }
                }

                //establish contraction order
                self.ordered_nodes.insert(node_id, index);
                total_shortcuts += self.contract_node(node_id, graph, false).0 as u64;
                //for (neighbor, _ ) in graph.graph.edges.get(&node_id).unwrap().clone() {
                //    let new_e_d  = self.contract_node(neighbor, graph, true).1;
                //    priority_queue.push(Reverse((new_e_d, node_id)));
                //}
                index += 1;
            }
            //step 3: reset arc flags, only flags of arc(u, v) with u.index < v.index == true
            graph.reset_all_flags(false);
            for (u, edgelist) in graph.graph.edges.iter_mut() {
                for (v, (_, flag)) in edgelist {
                    if self.ordered_nodes.get(u) < self.ordered_nodes.get(v) {
                        *flag = true;
                    }
                }
            }
            graph.set_max_settled_nodes(u64::MAX);
            graph.set_cost_upper_bound(31536000);
            total_shortcuts
        }

        pub fn bidirectional_compute(
            graph: &mut RoadDijkstra,
            source_id: i64,
            target_id: i64,
        ) -> (u64, i64) {
            graph.dijkstra(source_id, target_id, &None, true);
            let dist_source_to_u = graph.visited_nodes.clone();
            graph.dijkstra(target_id, source_id, &None, true);
            let dist_target_to_u = &graph.visited_nodes;

            let mut distances_s_t = Vec::new();
            for (u, dist_s) in dist_source_to_u {
                if let Some(dist_t) = dist_target_to_u.get(&u) {
                    distances_s_t.push((dist_t + dist_s, u));
                }
            }
            *distances_s_t.iter().min().unwrap_or(&(0, 0))
        }
    }
}

pub mod landmark_algo {
    use crate::road_dijkstras::*;
    use std::collections::HashMap;

    pub fn landmark_heuristic_precompute(
        dijkstra_graph: &mut RoadDijkstra,
        num_landmarks: usize,
    ) -> HashMap<i64, HashMap<i64, u64>> {
        let roads = dijkstra_graph.graph.clone();
        let mut landmarks = Vec::new();
        for _ in 0..num_landmarks {
            landmarks.push(dijkstra_graph.get_random_node_id().unwrap());
        }
        let mut graph = RoadDijkstra::new(&roads);
        landmarks
            .iter()
            .map(|&l| {
                (l, {
                    graph.dijkstra(l, -1, &None, false);
                    graph
                        .visited_nodes
                        .iter()
                        .map(|(id, dist)| (*id, *dist))
                        .collect()
                })
            })
            .collect::<HashMap<i64, HashMap<i64, u64>>>() //landmark_id, node_id, distance
    }

    pub fn landmark_heuristic(
        landmark_precompute: &HashMap<i64, HashMap<i64, u64>>,
        dijkstra_graph: &RoadDijkstra,
        target: i64,
    ) -> HashMap<i64, u64> {
        dijkstra_graph
            .graph
            .nodes
            .keys()
            .map(|source| {
                (*source, {
                    landmark_precompute
                        .values()
                        .map(|arr| {
                            let dist_lu = *arr.get(source).unwrap();
                            let dist_tu = *arr.get(&target).unwrap();
                            dist_lu.abs_diff(dist_tu)
                        })
                        .max()
                        .unwrap()
                })
            })
            .collect()
    }
}

