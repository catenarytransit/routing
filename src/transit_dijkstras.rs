//transit_dijkstras algorithms and helper functiions
use crate::transit_network::*;
use crate::NodeType;
use gtfs_structures::Trip;
use rand::Rng;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::hash::Hash;

#[derive(Debug, PartialEq, Clone, Eq, PartialOrd, Ord, Hash, Default)]
pub struct PathedNode {
    //node that references parent nodes, used to create path from goal node to start node
    pub cost_from_start: u32,
    pub parent_node: Option<NodeId>,
    pub transfer_count: u8,
}

impl PathedNode {
    //some wacky linked-list kind struct

    pub fn update(
        &mut self,
        id: NodeId,
        new_cost: u32,
        parent: NodeId,
        transfers: u8,
        gscore: &mut HashMap<NodeId, u32>,
        priority_queue: &mut BinaryHeap<Reverse<(u32, NodeId)>>,
    ) {
        let temp_distance = self.cost_from_start + new_cost;
        let next_distance = *gscore.get(&id).unwrap_or(&u32::MAX);

        if temp_distance < next_distance {
            gscore.insert(id, temp_distance);

            self.cost_from_start += new_cost;
            self.parent_node = Some(parent);
            self.transfer_count += transfers;

            priority_queue.push(Reverse((temp_distance, id)));
        }
    }

    pub fn get_path(id: NodeId, nodespace: &HashMap<NodeId, PathedNode>) -> (Vec<NodeId>, u32) {
        //path, cost
        //uses reference to find the source node with parent_node == None
        //vec.get(0) = target node
        let mut shortest_path = Vec::new();
        let pathed_node = nodespace.get(&id).unwrap();
        let total_distance: u32 = pathed_node.cost_from_start;
        let mut current = pathed_node;
        while let Some(prev) = current.parent_node {
            shortest_path.push(id); //current.node_self
            let next_node = nodespace.get(&prev).unwrap();
            current = next_node; //current = current.parent_node
        }
        //shortest_path.push(current_path.unwrap()));
        (shortest_path, total_distance)
    }

    pub fn get_tp(
        self,
        id: NodeId,
        nodespace: &HashMap<NodeId, PathedNode>,
        trips: &HashMap<String, Trip>,
    ) -> (Vec<(NodeId, Option<String>)>, u32) {
        let mut tp = Vec::new();
        let journey_cost: u32 = self.cost_from_start;
        let mut current = &self;
        let mut prev_node: Option<NodeId> = None;
        let mut prev_route: Option<String> = None;
        while let Some(prev) = current.parent_node {
            let node = id;
            let route = current.get_route(id, trips);
            let next_node = nodespace.get(&prev).unwrap();
            current = next_node;

            if let Some(prev) = prev_node {
                if prev.trip_id != node.trip_id && prev_route != route
                //&& !(prev.node_type == NodeType::Transfer && prev.station_id == node.station_id)
                {
                    tp.push((node, route.to_owned()));
                }
            } else {
                tp.push((node, route.to_owned()));
            }

            prev_route = route;
            prev_node = Some(node);
        }
        tp.reverse();
        (tp, journey_cost)
    }

    pub fn get_route(&self, id: NodeId, trips: &HashMap<String, Trip>) -> Option<String> {
        let trip_id = id.trip_id.to_string();
        let try_trip = trips.get(&trip_id);
        if let Some(trip) = try_trip {
            return Some(trip.route_id.clone());
        }
        
        None
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct TransitDijkstra {
    //handle time expanded dijkstra calculations
    pub graph: TimeExpandedGraph,
    cost_upper_bound: u32,
}

impl TransitDijkstra {
    //implementation of time expanded dijkstra's shortest path algorithm
    pub fn new(graph: TimeExpandedGraph) -> (Self, HashMap<NodeId, PathedNode>) {
        let t_graph = graph.clone();
        let mut paths = HashMap::new();
        for node in graph.nodes {
            paths.insert(node, PathedNode::default());
        }
        (
            Self {
                graph: t_graph,
                cost_upper_bound: u32::MAX,
            },
            paths,
        )
    }

    pub fn set_cost_upper_bound(&mut self, upper_bound: u32) {
        self.cost_upper_bound = upper_bound;
    }

    pub fn get_neighbors(
        &self,
        current: &NodeId,
        visited_nodes: &HashSet<NodeId>,
    ) -> Vec<(NodeId, u32)> {
        //return node id of neighbors
        let mut paths = Vec::new();
        if let Some(connections) = self.graph.edges.get(current) {
            for (next_node_id, cost) in connections {
                if visited_nodes.contains(next_node_id) {
                    continue;
                }

                paths.push((*next_node_id, *cost));
            }
        }
        paths
    }

    pub fn time_expanded_dijkstra(
        &self,
        source_id_set: Vec<NodeId>,
        hubs: Option<&HashSet<i64>>,
        paths: &mut HashMap<NodeId, PathedNode>,
    ) -> HashSet<NodeId> {
        //path, visted nodes, transfer count
        //returns path from the source to target if exists, also path from every node to source
        //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)

        let mut priority_queue: BinaryHeap<Reverse<(u32, NodeId)>> = BinaryHeap::new();
        let mut visited_nodes: HashSet<NodeId> = HashSet::new();
        //let mut inactive_nodes: HashSet<NodeId> = HashSet::new();

        //stores distances of node relative to target
        let mut gscore: HashMap<NodeId, u32> = HashMap::new();
        for source_id in source_id_set {
            gscore.insert(source_id, 0);
            priority_queue.push(Reverse((0, source_id)));
        }

        let mut pathed_node;

        while !priority_queue.is_empty() {
            let current_node = priority_queue.pop().unwrap().0 .1; //.0 "unwraps" from Reverse()
            pathed_node = paths.get(&current_node).unwrap();

            visited_nodes.insert(current_node);

            //stop conditions
            //cost or # of settled nodes goes over limit
            if pathed_node.cost_from_start > self.cost_upper_bound {
                return visited_nodes;
            }

            //cost is higher than current path (not optimal)
            if pathed_node.cost_from_start > *gscore.get(&current_node).unwrap_or(&u32::MAX) {
                continue;
            }

            let neighbors = self.get_neighbors(&current_node, &visited_nodes);

            for neighbor in neighbors {
                let mut transfer_count = 0;
                if current_node.node_type == NodeType::Transfer
                    //&& neighbor.0.node_type == NodeType::Arrival
                    && neighbor.0.node_type == NodeType::Departure
                {
                    //transfer arc detected, increment transfer count for current path
                    transfer_count += 1;

                    //limit local search to at most 2 transfers for 3-legs heuristic
                    if transfer_count > 2 && hubs.is_some() {
                        continue;
                    }
                }

                paths.entry(neighbor.0).and_modify(|n| {
                    n.update(
                        neighbor.0,
                        neighbor.1,
                        current_node,
                        transfer_count,
                        &mut gscore,
                        &mut priority_queue,
                    )
                });
            }
        }
        //println!("no path exists");
        visited_nodes
    }

    pub fn get_random_node_id(&self) -> Option<NodeId> {
        //returns ID of a random valid node from a graph
        let full_node_list = self.graph.nodes.iter().copied().collect::<Vec<NodeId>>();
        let mut rng = rand::rng();
        let random: usize = rng.random_range(0..full_node_list.len());
        full_node_list.get(random).copied()
    }

    /*pub fn get_random_start(&self) -> Option<NodeId> {
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
    }*/
}

#[derive(Debug, PartialEq, Clone)]
pub struct TDDijkstra {
    //handle time dependent dijkstra calculations
    pub connections: DirectConnections,
    pub edges: HashMap<NodeId, HashSet<NodeId>>,
    pub visited_nodes: HashSet<NodeId>,
}

impl TDDijkstra {
    //implementation of time dependent shortest path algorithm
    pub fn new(connections: DirectConnections, edges: HashMap<NodeId, HashSet<NodeId>>) -> Self {
        let visited_nodes = HashSet::new();
        Self {
            connections,
            edges,
            visited_nodes,
        }
    }

    pub fn get_neighbors(
        &self,
        current: &NodeId,
        connections: &DirectConnections,
    ) -> Vec<(NodeId, u32)> {
        //return node id of neighbors
        let mut paths = Vec::new();

        if let Some(arcs) = self.edges.get(current) {
            for next_node in arcs {
                if self.visited_nodes.contains(next_node) {
                    continue;
                }

                if let Some((dept, arr)) = direct_connection_query(
                    connections,
                    current.station_id,
                    next_node.station_id,
                    current.time.unwrap(),
                ) {
                    let cost = arr - dept;
                    paths.push((*next_node, cost));
                }
            }
        }
        paths
    }

    pub fn time_dependent_dijkstra(
        &mut self,
        paths: &mut HashMap<NodeId, PathedNode>,
        source_id: NodeId,
        target_id: &HashSet<NodeId>, //if target == None, settles all reachable nodes
    ) -> Option<NodeId> {
        //returns path from the source to target if exists, also path from every node to source
        //Heap(distance, node), Reverse turns binaryheap into minheap (default is maxheap)

        let mut priority_queue: BinaryHeap<Reverse<(u32, NodeId)>> = BinaryHeap::new();
        let mut gscore: HashMap<NodeId, u32> = HashMap::new(); //stores distances of node relative to target
        self.visited_nodes.clear();

        gscore.insert(source_id, 0);
        priority_queue.push(Reverse((0, source_id)));

        let mut pathed_node;

        while !priority_queue.is_empty() {
            let current_node = priority_queue.pop().unwrap().0 .1;
            pathed_node = paths.get(&current_node).unwrap();

            self.visited_nodes.insert(current_node);

            //found target node
            if target_id.contains(&current_node) {
                return Some(current_node);
            }

            //cost is higher than current path (not optimal)
            if pathed_node.cost_from_start > *gscore.get(&current_node).unwrap_or(&u32::MAX) {
                continue;
            }

            for neighbor in self.get_neighbors(&current_node, &self.connections) {
                paths.entry(neighbor.0).and_modify(|n| {
                    n.update(
                        neighbor.0,
                        neighbor.1,
                        current_node,
                        0,
                        &mut gscore,
                        &mut priority_queue,
                    )
                });
            }
        }
        None //(None, node_path_tracker)
    }
}
