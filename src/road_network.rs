#[allow(unused)]
pub mod road_graph_construction {
    //constructs and preprocesses the graph struct from OSM data
    use crate::road_dijkstras::*;
    use core::num;
    use osmpbfreader::objects::OsmObj;
    use serde::{Deserialize, Serialize};
    use std::{collections::HashMap, ops::Index};

    #[derive(Debug, PartialEq, Hash, Eq, Clone, Copy, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
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

    pub fn speed_calc(highway: &str) -> Option<u16> {
        //for pedestrians
        //picks speed of highway based on given values, in km/h
        match highway {            
            /* "pedestrian" => Some(4),
            "path" => Some(4),
            "footway" => Some(4),
            "steps" => Some(4),
            "corridor" => Some(4),
            "living_street" => Some(4),
            "sidewalk" => Some(4),
            "traffic_island" => Some(4),
            "crossing" => Some(3),
            "road" => Some(4),
            "unclassified" => Some(4),
            "residential" => Some(4),
            "unsurfaced" => Some(4),
            "living_street" => Some(4), */
            "motorway" => None,
            "service" => Some(4),
            "trunk" => Some(4),
            "primary" => Some(4),
            "secondary" => Some(4),
            "tertiary" => Some(4),
            "motorway_link" => Some(4),
            "trunk_link" => Some(4),
            "primary_link" => Some(4),
            "secondary_link" => Some(4),
            _ => Some(4),
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
                        let cost = (c as u64) / ((way.speed as f64) * 5.0 / 18.0) as u64; //seconds needed to traverse segment based on road type
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

        pub fn read_from_osm_file(path: &str) -> Option<(HashMap<i64, Node>, Vec<Way>)> {
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
                        {
                            if let Some(speed) = speed_calc(road_type.1.as_str()) {
                                ways.push(Way {
                                    id: e.id.0,
                                    speed,
                                    refs: e.nodes.into_iter().map(|x| x.0).collect(),
                                });
                            }
                        }
                    }
                    _ => {}
                }
            }
            Some((nodes, ways))
        }

        pub fn reduce_to_largest_connected_component(self) -> Self {
            //reduces graph to largest connected component through nodes visited with dijkstra
            let mut counter = 0;
            let mut number_times_node_visted: HashMap<i64, i32> = HashMap::new();
            let mut shortest_path_graph = RoadDijkstra::new(&self);
            let mut max_connections = 0;

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
                .collect::<HashMap<i64, Node>>();

            RoadNetwork::new(lcc_nodes, self.raw_ways)
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
            .iter()
            .map(|(source, _)| {
                (*source, {
                    landmark_precompute
                        .iter()
                        .map(|(_, arr)| {
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

pub mod arc_flags_algo {
    use crate::road_dijkstras::*;
    use core::ops::Range;
    use std::collections::HashSet;

     #[derive(Debug, PartialEq, Clone, serde::Serialize, serde::Deserialize)]
    pub struct ArcFlags {
        //precomputation stuff for arc flag routing algorithm
        pub lat_range: Range<i64>,
        pub lon_range: Range<i64>,
    }
    #[allow(dead_code)]
    impl ArcFlags {
        pub fn new(lat_min: f32, lat_max: f32, lon_min: f32, lon_max: f32) -> ArcFlags {
            ArcFlags {
                lat_range: (lat_min * f32::powi(10.0, 7)) as i64
                    ..(lat_max * f32::powi(10.0, 7)) as i64,
                lon_range: (lon_min * f32::powi(10.0, 7)) as i64
                    ..(lon_max * f32::powi(10.0, 7)) as i64,
            }
        }

        pub fn arc_flags_precompute(self, dijkstra_graph: &mut RoadDijkstra) {
            let mut boundary_node = HashSet::new();
            let region_nodes = dijkstra_graph
                .graph
                .nodes
                .iter()
                .filter(|(_, &node)| {
                    self.lat_range.contains(&node.lat) && self.lon_range.contains(&node.lon)
                })
                .map(|(id, _)| *id)
                .collect::<Vec<i64>>();

            for node in region_nodes.clone() {
                if let Some(edge_list) = dijkstra_graph.graph.edges.get_mut(&node) {
                    for edge in edge_list.iter_mut() {
                        if region_nodes.contains(edge.0) {
                            edge.1 .1 = true;
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
        }
    }
}

#[allow(dead_code)]
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
