//routing algorithms and helper functiions
use crate::RoadNetwork;
use rand::Rng;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::hash::Hash;
use std::rc::Rc;

use crate::road_network::road_graph_construction::Node;

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
        let total_distance: u64 = self.distance_from_start;
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
            if consider_arc_flags && !path.1 .1 {
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
    ) -> Option<RoadPathedNode> { //(Option<RoadPathedNode>, HashMap<i64, i64>) {
        //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
        let mut priority_queue: BinaryHeap<Reverse<(u64, RoadPathedNode)>> = BinaryHeap::new();
        //let mut previous_nodes = HashMap::new();

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

        let mut cost;
        while !priority_queue.is_empty() {
            let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
            cost = pathed_current_node.distance_from_start;
            let idx = pathed_current_node.node_self.id;

            self.visited_nodes.insert(idx, cost);

            //found target node
            if idx.eq(&target_id) {
                return Some(pathed_current_node)
                //return (Some(pathed_current_node), previous_nodes);
            }

            //stop conditions
            //cost or # of settled nodes goes over limit
            if cost > self.cost_upper_bound
                || self.visited_nodes.len() > self.max_settled_nodes as usize
            {
                return None
                //return (None, previous_nodes);
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
                    //previous_nodes.insert(neighbor.0.id, pathed_current_node.node_self.id);
                }
            }
        }
        None
        //(None, previous_nodes)
    }

    pub fn get_random_node_id(&mut self) -> Option<i64> {
        //returns ID of a random valid node from a graph
        let mut rng = rand::thread_rng();
        let full_node_list = &self.graph.raw_nodes;
        let random: usize = rng.gen_range(0..full_node_list.len());
        let node_id = full_node_list.get(random).unwrap();

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
        while !found {
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
            print!("\tall nodes visted\t");
            return None;
        }
        let other_located_nodes = other_located_nodes
            .iter()
            .filter(|(_, count)| **count > 0)
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
