#[allow(unused)]
mod graph_construction {
    //constructs and preprocesses the graph struct from OSM data
    use crate::routing::*;
    use core::num;
    use osmpbfreader::objects::OsmObj;
    use std::{collections::HashMap, ops::Index};

    #[derive(Debug, PartialEq, Hash, Eq, Clone, Copy, PartialOrd, Ord)]
    pub struct Node {
        //nodes from OSM, each with unique ID and coordinate position
        pub id: i64,
        pub lat: i64,
        pub lon: i64,
    }

    #[derive(Debug, PartialEq, Hash, Eq, Clone)]
    pub struct Way {
        //ways from OSM, each with unique ID, speed from highway type, and referenced nodes that it connects
        pub id: i64,
        pub speed: u64,
        pub refs: Vec<i64>,
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct RoadNetwork {
        //graph struct that will be used to route
        pub nodes: HashMap<i64, Node>, // <node.id, node>
        pub edges: HashMap<i64, HashMap<i64, (u64, bool)>>, // tail.id, <head.id, (cost, arcflag)>
        pub raw_ways: Vec<Way>,
        pub raw_nodes: Vec<i64>,
    }

    fn speed_calc(highway: &str) -> Option<u64> {
        //calculates speed of highway based on given values
        match highway {
            "motorway" => Some(110),
            "trunk" => Some(110),
            "primary" => Some(70),
            "secondary" => Some(60),
            "tertiary" => Some(50),
            "motorway_link" => Some(50),
            "trunk_link" => Some(50),
            "primary_link" => Some(50),
            "secondary_link" => Some(50),
            "road" => Some(40),
            "unclassified" => Some(40),
            "residential" => Some(30),
            "unsurfaced" => Some(30),
            "living_street" => Some(10),
            "service" => Some(5),
            _ => None,
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
            let mut shortest_path_graph = Dijkstra::new(&self);
            let mut max_connections = 0;

            while let Some(source_id) =
                shortest_path_graph.get_unvisted_node_id(&number_times_node_visted)
            {
                counter += 1;
                let mut shortest_path_graph = Dijkstra::new(&self);
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
        pub graph: RoadNetwork,
        pub visited_nodes: HashMap<i64, u64>,
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

    impl Dijkstra {
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
            source_id: i64,
            target_id: i64,
            heuristics: &Option<HashMap<i64, u64>>,
            consider_arc_flags: bool,
        ) -> (Option<PathedNode>, HashMap<i64, i64>) {
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

        pub fn backtrace_dijkstra(
            &mut self,
            source_id: i64,
            target_id: i64,
            heuristics: &Option<HashMap<i64, u64>>,
            consider_arc_flags: bool,
        ) -> HashSet<PathedNode> {
            //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
            let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();
            let mut v_nodes: HashSet<PathedNode> = HashSet::new();

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
                    return v_nodes;
                }

                let neighbors = self.get_neighbors(&pathed_current_node, consider_arc_flags);
                for neighbor in neighbors {
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
                        v_nodes.insert(tentative_new_node.clone());

                        let h;
                        if let Some(heuristic) = heuristics {
                            h = heuristic.get(&neighbor.0.id).unwrap_or(&0);
                        } else {
                            h = &0;
                        }

                        //fscore = temp_distance (gscore) + h (hscore)
                        priority_queue.push(Reverse((temp_distance + h, tentative_new_node)));
                    }
                }
                counter += 1;
            }
            v_nodes
        }
    }
}

fn main() {}

#[cfg(test)]
mod tests {
    //use crate::graph_construction::*;
    //use crate::routing::*;
    //use std::collections::HashMap;
    //use std::time::Instant;

    /*
    #[test]
    fn tnr_test_custom() {
        let node0 = Node { id: 0, lat: 0, lon: 0 };
        let node1 = Node { id: 1, lat: 0, lon: 0 };
        let node2 = Node { id: 2, lat: 0, lon: 0 };
        let node3 = Node { id: 3, lat: 0, lon: 0 };
        let node4 = Node { id: 4, lat: 0, lon: 0 };
        let node5 = Node { id: 5, lat: 0, lon: 0 };
        let node6 = Node { id: 6, lat: 0, lon: 0 };
        let node7 = Node { id: 7, lat: 0, lon: 0 };
        let node8 = Node { id: 8, lat: 0, lon: 0 };
        let node9 = Node { id: 9, lat: 0, lon: 0 };

        let roads = RoadNetwork {
            nodes: HashMap::from([ (0, node0), (1, node1), (2, node2), (3, node3), (4, node4), (5, node5), (6, node6), (7, node7), (8, node8), (9, node9)]),
            edges: HashMap::from([
                (0, HashMap::from([ (1, (3, false)) ])),
                (1, HashMap::from([ (2, (3, false)), (3, (3, false)), (8, (3, false)), (0, (3, false)) ])),
                (2, HashMap::from([ (1, (3, false)), (4, (3, false)), (5, (3, false)) ])),
                (3, HashMap::from([ (1, (3, false)), (6, (3, false)), (7, (3, false)) ])),
                (4, HashMap::from([ (2, (3, false)) ])),
                (5, HashMap::from([ (2, (3, false)) ])),
                (6, HashMap::from([ (3, (3, false)) ])),
                (7, HashMap::from([ (3, (3, false)) ])),
                (8, HashMap::from([ (1, (3, false)), (9, (3, false)) ])),
                (9, HashMap::from([ (8, (3, false)) ]))
            ]),
            raw_ways: vec![Way{id:0,speed:0, refs:vec![0,0]}],
            raw_nodes: vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
        };

        let mut graph = Dijkstra::new(&roads);

        graph.reset_all_flags(true);
        let transit_nodes = ch_based_precompute(&mut graph, 3);
        //let transit_nodes = HashSet::from([2, 3, 8]);
        println!("{:?}", transit_nodes);

        for _ in 0..1 {
            let source_id = graph.get_random_node_id().unwrap();
            let target_id = graph.get_random_node_id().unwrap();
            println!("s: {}, t: {}", source_id, target_id);
            let access_nodes = compute_access_nodes(source_id, &mut graph, &transit_nodes);
            print!("n:  {:?}\t", access_nodes);
            let distance_sets = access_node_distance_generator(source_id, target_id, access_nodes, &mut graph);
            let result = compute_shortest_path_tnr(distance_sets);

            println!("{}", result)
        }

    }
    */
}
