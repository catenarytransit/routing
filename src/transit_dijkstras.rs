//transit_dijkstras algorithms and helper functiions
use crate::transit_network::*;
use crate::NodeType;
use rand::Rng;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::hash::Hash;
use std::sync::Arc;

#[derive(Debug, PartialEq, Clone)]
pub struct TransitDijkstra {
    //handle time expanded dijkstra calculations
    pub graph: TimeExpandedGraph,
    cost_upper_bound: u64,
    inactive_nodes: HashSet<NodeId>,
}

#[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord, Hash)]
pub struct PathedNode {
    //node that references parent nodes, used to create path from goal node to start node
    pub node_self: NodeId,
    pub cost_from_start: u64,
    pub parent_node: Option<Arc<PathedNode>>,
    pub transfer_count: u8,
}

impl PathedNode {
    pub fn get_path(self) -> (Vec<NodeId>, u64) {
        //path, cost
        //uses reference to find the source node with parent_node == None
        //vec.get(0) = target node
        let mut shortest_path = Vec::new();
        let total_distance: u64 = self.cost_from_start;
        let mut current_path = &Some(Arc::new(self));
        while let Some(current) = current_path {
            shortest_path.push(current.node_self); //current.node_self
            current_path = &current.parent_node; //current = current.parent_node
        }
        //shortest_path.push(current_path.unwrap()));
        (shortest_path, total_distance)
    }
}

impl TransitDijkstra {
    //implementation of time expanded dijkstra's shortest path algorithm
    pub fn new(graph: &TimeExpandedGraph) -> Self {
        let inactive_nodes = HashSet::new();
        Self {
            graph: graph.clone(),
            cost_upper_bound: u64::MAX,
            inactive_nodes,
        }
    }

    pub fn set_cost_upper_bound(&mut self, upper_bound: u64) {
        self.cost_upper_bound = upper_bound;
    }

    pub fn get_neighbors(
        &self,
        current: &PathedNode,
        visited_nodes: &HashMap<NodeId, PathedNode>,
        is_local: bool,
    ) -> Vec<(NodeId, u64)> {
        //return node id of neighbors
        let mut paths = Vec::new();
        if let Some(connections) = self.graph.edges.get(&current.node_self) {
            for (next_node_id, cost) in connections {
                if visited_nodes.contains_key(next_node_id) {
                    continue;
                }
                
                if current.transfer_count >= 2
                    && current.node_self.node_type == NodeType::Transfer
                    && next_node_id.node_type == NodeType::Departure
                    && is_local
                {
                    //number of transfers exceeds 2 if this path is followed, so ignore it for the 3-legs heuristic
                    continue;
                }

                paths.push((*next_node_id, *cost));
            }
        }
        paths
    }

    pub fn node_deactivator(&mut self, hubs: &HashSet<i64>) {
        for (u, edges) in self.graph.edges.iter() {
            if u.node_type == NodeType::Transfer && hubs.contains(&u.station_id) {
                for v in edges.keys() {
                    self.inactive_nodes.insert(*v);
                }
            } else {
                for v in edges.keys() {
                    if v.node_type == NodeType::Transfer && hubs.contains(&v.station_id) {
                        self.inactive_nodes.insert(*u);
                    }
                }
            }
        }
    }

    pub fn time_expanded_dijkstra(
        &self,
        source_id: Option<NodeId>,
        source_id_set: Option<Vec<NodeId>>,
        target_id: Option<NodeId>, //if target == None, settles all reachable nodes
        hubs: Option<&HashSet<i64>>,
    ) -> (Option<PathedNode>, HashMap<NodeId, PathedNode>) {
        //path, visted nodes, transfer count
        //returns path from the source to target if exists, also path from every node to source
        //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)
        let mut priority_queue: BinaryHeap<Reverse<(u64, PathedNode)>> = BinaryHeap::new();
        let mut visited_nodes: HashMap<NodeId, PathedNode> = HashMap::new();

        //stores distances of node relative to target
        let mut gscore: HashMap<NodeId, u64> = HashMap::new();

        //resets list of settled nodes for new computation
        visited_nodes.clear();

        let source_id = self.graph.nodes.get(&source_id.unwrap());
        if source_id.is_none() {
            println!("s wth");
        }
        let target_id = self.graph.nodes.get(&target_id.unwrap());
        if target_id.is_none() {
            println!("t wth");
        }

        if let Some(source_id) = source_id {
            let source_node: PathedNode = PathedNode {
                node_self: (*source_id),
                cost_from_start: 0,
                parent_node: (None),
                transfer_count: 0,
            };

            gscore.insert(*source_id, 0);

            priority_queue.push(Reverse((0, source_node)));
        } else if let Some(source_id_set) = source_id_set {
            for source_id in source_id_set {
                let source_node: PathedNode = PathedNode {
                    node_self: source_id,
                    cost_from_start: 0,
                    parent_node: None,
                    transfer_count: 0,
                };

                gscore.insert(source_id, 0);
                priority_queue.push(Reverse((0, source_node)));
            }
        }


        let mut current_cost;
        //let mut num_visited_inactive = 0;

        while !priority_queue.is_empty() {
            let pathed_current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
            current_cost = pathed_current_node.cost_from_start;
            let idx = pathed_current_node.node_self;

            /*if self.inactive_nodes.contains(&pathed_current_node.node_self) {
                num_visited_inactive += 1
            }*/

            visited_nodes.insert(idx, pathed_current_node.clone());

            //found target node
            if let Some(target_id) = target_id {
                if idx.eq(target_id) {
                    return (Some(pathed_current_node), visited_nodes);
                }
            }

            //stop search for local TP if all unsettled NodeIds are inactive -->
            //all unvisited nodes should become subset of inactive nodes
            //this cool math solution was thought of by a server-mate on Discord, thank you! 
            //NEW NOTE: THIS METHOD MIGHT NOT WORK, NEED TO CHECK ON NODE-BY-NODE CASE
            /*if hubs.is_some()
                && (self.graph.nodes.len()
                    - visited_nodes.len()
                    - (self.inactive_nodes.len() - num_visited_inactive)
                    == 0)
            {
                return (None, visited_nodes);
            }*/

            //stop conditions
            //cost or # of settled nodes goes over limit
            if current_cost > self.cost_upper_bound {
                println!("cost over");
                return (None, visited_nodes);
            }

            //cost is higher than current path (not optimal)
            if current_cost > *gscore.get(&idx).unwrap_or(&u64::MAX) {
                continue;
            }

            for neighbor in self.get_neighbors(&pathed_current_node, &visited_nodes, hubs.is_some())
            {
                let temp_distance = current_cost + neighbor.1;
                let next_distance = *gscore.get(&neighbor.0).unwrap_or(&u64::MAX);

                if temp_distance < next_distance {
                    gscore.insert(neighbor.0, temp_distance);
                    let prev_node: Arc<PathedNode> = Arc::new(pathed_current_node.clone());
                    let mut transfer_count = pathed_current_node.transfer_count;
                    if pathed_current_node.node_self.node_type == NodeType::Transfer
                        && neighbor.0.node_type == NodeType::Departure
                    {
                        //transfer arc detected, increment transfer count for current path
                        transfer_count += 1;
                    }
                    let tentative_new_node = PathedNode {
                        node_self: neighbor.0,
                        cost_from_start: temp_distance,
                        parent_node: Some(prev_node),
                        transfer_count,
                    };

                    priority_queue.push(Reverse((temp_distance, tentative_new_node)));
                }
            }
        }
        println!("no path exists");
        (None, visited_nodes)
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
            .filter(|id| {
                id.node_type == NodeType::Departure
                    && id.time > Some(21600)
                    && id.time < Some(75600)
            })
            .copied()
            .collect();
        let mut rng = rand::thread_rng();
        let random: usize = rng.gen_range(0..full_node_list.len());
        full_node_list.get(random).copied()
    }
}
#[derive(Debug, PartialEq, Clone)]
pub struct TDDijkstra {
    //handle time dependentdijkstra calculations
    pub connections: DirectConnections,
    pub edges: HashMap<NodeId, Vec<NodeId>>,
    pub visited_nodes: HashMap<NodeId, PathedNode>,
}

impl TDDijkstra {
    //implementation of time dependent shortest path algorithm
    pub fn new(connections: DirectConnections, edges: HashMap<NodeId, Vec<NodeId>>) -> Self {
        println!("dependent edges: {:?}", edges.len());

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

    pub fn time_dependent_dijkstra(
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

        let mut current_cost = 0;

        let source_node: PathedNode = PathedNode {
            node_self: (source_id),
            cost_from_start: current_cost,
            parent_node: (None),
            transfer_count: 0,
        };

        gscore.insert(source_id, 0);

        priority_queue.push(Reverse((0, source_node)));

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
                    let prev_node: Arc<PathedNode> = Arc::new(pathed_current_node.clone());
                    let tentative_new_node = PathedNode {
                        node_self: neighbor.0,
                        cost_from_start: temp_distance,
                        parent_node: Some(prev_node),
                        transfer_count: 0,
                    };

                    priority_queue.push(Reverse((temp_distance, tentative_new_node.clone())));
                }
            }
        }
        None //(None, node_path_tracker)
    }
}
