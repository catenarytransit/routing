//THE FINAL BOSS
use crate::{road_dijkstras::*, transit_dijkstras::*, transit_network::*};
use geo::algorithm::haversine_distance::*;
use geo::point;
use geo::Point;
use std::cmp::Reverse;
use std::collections::hash_map::Entry;
use std::collections::{BinaryHeap, HashMap, HashSet};
//use std::time::Instant;
use std::sync::Arc;
//use std::sync::Mutex;
//use std::thread;
use rstar::*;

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
            transfer_count: 0,
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
                    let prev_node: Arc<PathedNode> = Arc::new(pathed_current_node.clone());
                    let tentative_new_node = PathedNode {
                        node_self: neighbor.0,
                        cost_from_start: temp_distance,
                        parent_node: Some(prev_node),
                        transfer_count: 0,
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

use crate::RoadNetwork;

//only calculate global time_expanded_dijkstra from hubs (important stations) to save complexity
//picks important hubs if they are more often visted from Dijkstras-until-all-nodes-settled
pub fn hub_selection(
    router: &TransitDijkstra,
    random_samples: u32,
    cost_limit: u64,
) -> HashSet<i64> {
    //station ids
    let num_stations = 1.max((router.graph.station_mapping.len() as u32) / 100);
    let mut selected_hubs: HashSet<i64> = HashSet::new();

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
                    }
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

    let mut time_independent_router = TransitDijkstra::new(&time_independent_graph);
    time_independent_router.set_cost_upper_bound(cost_limit);

    let mut hub_list: HashMap<NodeId, u16> = HashMap::new();

    for _ in 0..random_samples {
        let current_node = time_independent_router.get_random_node_id();
        time_independent_router.time_expanded_dijkstra(current_node, None, None, None);
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

    let mut sorted_hubs: BinaryHeap<(u16, NodeId)> =
        hub_list.into_iter().map(|(n, c)| (c, n)).collect();

    for _ in 0..num_stations {
        let hub = sorted_hubs.pop().unwrap();
        selected_hubs.insert(hub.1.station_id);
    }
    selected_hubs
}

// Precompute transfer patterns from a given station to all other stations.
// Return the transfer patterns & numbers of ybetween each station pair.
pub fn num_transfer_patterns_from_source(
    source_station_id: i64,
    router: &mut TransitDijkstra,
    hubs: Option<&HashSet<i64>>,
) -> HashMap<(NodeId, NodeId), Vec<NodeId>> {
    let source_transfer_nodes: Option<Vec<NodeId>> = Some(
        router
            .graph
            .nodes_per_station
            .get(&source_station_id)
            .unwrap()
            .iter()
            .filter(|(_, node)| node.node_type == 2) // || node.node_type == 1
            //must check for transfer nodes, but checking for arrival nodes may improve query time at expense of longer precompute
            .map(|(_, node)| *node)
            .collect(),
    );

    //note: multilabel time_expanded_dijkstras are always slower due to label set maintenance
    router.time_expanded_dijkstra(None, source_transfer_nodes, None, hubs);

    transfer_patterns_to_target(router)
}

// Backtrace all paths from a given station pair with respect to last Dijkstra
// computation. Return distinct set of the pair's transfer patterns
pub fn transfer_patterns_to_target(
    router: &mut TransitDijkstra,
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

    arrival_loop(&mut arrival_nodes);

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

// Arrival chain algo: For each arrival node, see if cost can be approved
// by simply waiting from an earlier arrival time. (favors less travel time)
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

pub fn stations_close_to_geo_point_and_time(
    search_point: &Point,
    preset_distance: &f64,
    graph: &TimeExpandedGraph,
    time: &u64,
) -> Vec<NodeId> {
    graph
            .nodes
            .iter()
            .filter(|node| {
                let node_coord = point!(x: node.lat as f64 / f64::powi(10.0, 14), y: node.lon as f64 / f64::powi(10.0, 14));
                search_point.haversine_distance(&node_coord) <= *preset_distance
                    && node.time >= Some(*time)
            })
            .copied()
            .collect()
}

pub fn query_graph_construction_from_geodesic_points(
    router: &mut TransitDijkstra,
    source: Point,
    target: Point,
    time: u64,
    preset_distance: f64, //in meters
) -> (Vec<NodeId>, Vec<NodeId>, HashMap<NodeId, Vec<NodeId>>) {
    //source nodes, target nodes, edges

    //let road_node_tree = RTree::bulk_load(router.graph.nodes.iter().map(|n| (n.lat, n.lon)).collect());

    //compute sets of N(source) and N(target) of stations N= near
    let sources =
        stations_close_to_geo_point_and_time(&source, &preset_distance, &router.graph, &time);

    println!("s len{}", sources.len());

    let targets =
        stations_close_to_geo_point_and_time(&target, &preset_distance, &router.graph, &time);

    println!("t targets{}", sources.len());

    //get hubs of important stations I(hubs)
    let hubs = hub_selection(router, 10000, 54000); //cost limit at 15 hours, arbitrary

    let mut total_transfer_patterns = HashMap::new();

    //precompute local TP from N(source) to first hub (this min hub is access station)
    //attempt at multithreading to go faster
    /*let source_chunk_len = sources.len();
    let threaded_sources = Arc::new(Mutex::new(sources.clone()));
    let total_transfer_patterns = Arc::new(Mutex::new(HashMap::new()));
    let arc_router = Arc::new(Mutex::new(router.clone()));
    let threaded_hubs = Arc::new(Mutex::new(hubs.clone()));
    let mut handles = vec![];

    for x in 1..5 {
        let source = Arc::clone(&threaded_sources);
        let transfer_patterns = Arc::clone(&total_transfer_patterns);
        let router = Arc::clone(&arc_router);
        let hub_list = Arc::clone(&threaded_hubs);
        let handle = thread::spawn(move || {
            let src = source.lock().unwrap();
            let mut ttp = transfer_patterns.lock().unwrap();
            for i in ((x - 1) * (source_chunk_len / 4))..(x * source_chunk_len / 4) {
                let source_id = src.get(i).unwrap();
                let l_tps = num_transfer_patterns_from_source(
                    source_id.station_id,
                    &mut router.lock().unwrap(),
                    Some(&hub_list.lock().unwrap()),
                );
                ttp.extend(l_tps.into_iter());
                println!("local tp for {i} is x`{:?}", ttp.len());
            }
        });


        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap();
    }

    //global transfer patterns from I(hubs) to to N(target())
    let hub_chunk_len = hubs.len();
    let total_transfer_patterns = Arc::new(Mutex::new(HashMap::new()));
    let arc_router = Arc::new(Mutex::new(router.clone()));
    let threaded_hubs = Arc::new(Mutex::new(hubs.into_iter().collect::<Vec<_>>()));
    let mut handles = vec![];

    for x in 1..5 {
        let transfer_patterns = Arc::clone(&total_transfer_patterns);
        let router = Arc::clone(&arc_router);
        let hub_list = Arc::clone(&threaded_hubs);
        let handle = thread::spawn(move || {
            let src = hub_list.lock().unwrap();
            for i in (x - 1) * (hub_chunk_len / 4)..(x * hub_chunk_len / 4) {
                let hub_id = src.get(i).unwrap();
                let g_tps = num_transfer_patterns_from_source(
                    *hub_id,
                    &mut router.lock().unwrap(),
                    None,
                );

                let mut ttp = transfer_patterns.lock().unwrap();
                ttp.extend(g_tps.into_iter());
            }
        });
        handles.push(handle);
    }

     for handle in handles {
        handle.join().unwrap();
    }

    println!("tp len {}", total_transfer_patterns.lock().unwrap().len());
    */

    for hub in hubs.iter() {
        let tps = num_transfer_patterns_from_source(*hub, router, None);
        total_transfer_patterns.extend(tps.into_iter());
    }
    println!("tps {}", total_transfer_patterns.len());

    router.node_deactivator(&hubs);

    let hubs = Some(hubs);
    for source in sources.iter() {
        let tps = num_transfer_patterns_from_source(source.station_id, router, hubs.as_ref());
        total_transfer_patterns.extend(tps.into_iter());
    }

    println!("tps {}", total_transfer_patterns.len());

    let mut raw_edges = HashMap::new();
    let _ = total_transfer_patterns
        //.lock()
        //.unwrap()
        .iter()
        .filter(|((source, target), _)| sources.contains(source) && targets.contains(target))
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

    println!(
        "length{}",
        total_transfer_patterns
            .iter()
            .filter(|((source, target), _)| sources.contains(source) && targets.contains(target))
            .collect::<Vec<_>>()
            .len()
    );

    (sources, targets, raw_edges)
}

use std::time::Instant;

pub fn query_graph_search(
    roads: RoadNetwork,
    connections: DirectConnections,
    edges: HashMap<NodeId, Vec<NodeId>>,
    start: Point,
    end: Point,
    sources: Vec<NodeId>,
    targets: Vec<NodeId>,
) -> Option<(NodeId, NodeId, PathedNode)> {
    let mut router = TDDijkstra::new(connections, edges);

    let mut source_paths: HashMap<&NodeId, Option<RoadPathedNode>> = HashMap::new();

    let time_rtree_insert = Instant::now();

    let road_node_tree = RTree::bulk_load(roads.nodes.values().map(|n| (n.lat, n.lon)).collect());

    println!("rtree insert time {:?}", time_rtree_insert.elapsed());

    println!("start final step");

    if let Some(start_road_node) = road_node_tree.nearest_neighbor(&(
        ((start.0.x * f64::powi(10.0, 14)) as i64),
        ((start.0.y * f64::powi(10.0, 14)) as i64),
    )) {
        print!("a\t");
        for source in sources.iter() {
            print!("b\t");
            let mut graph = RoadDijkstra::new(&roads);
            if let Some(station_sought) = road_node_tree.nearest_neighbor(&(source.lat, source.lon))
            {
                print!("c\t");
                let road_source = roads
                    .nodes
                    .values()
                    .find(|n| n.lat == start_road_node.0 && n.lon == start_road_node.1)
                    .unwrap();
                let station = roads
                    .nodes
                    .values()
                    .find(|n| n.lat == station_sought.0 && n.lon == station_sought.1)
                    .unwrap();

                let result = graph.dijkstra(road_source.id, station.id, &None, false);
                source_paths.insert(source, result.0);
            }
        }
    }

    println!("\tsource paths {}", source_paths.len());

    let mut target_paths: HashMap<&NodeId, Option<RoadPathedNode>> = HashMap::new();
    //note: remember to write a function that returns the closest approx point from road network if cannot find a point!
    if let Some(end_road_node) = road_node_tree.nearest_neighbor(&(
        ((end.0.x * f64::powi(10.0, 14)) as i64),
        ((end.0.y * f64::powi(10.0, 14)) as i64),
    )) {
        for target in targets.iter() {
            let mut graph = RoadDijkstra::new(&roads);
            if let Some(station_sought) = road_node_tree.nearest_neighbor(&(target.lat, target.lon))
            {
                let road_target = roads
                    .nodes
                    .values()
                    .find(|n| n.lat == end_road_node.0 && n.lon == end_road_node.1)
                    .unwrap();
                let station = roads
                    .nodes
                    .values()
                    .find(|n| n.lat == station_sought.0 && n.lon == station_sought.1)
                    .unwrap();

                let result = graph.dijkstra(station.id, road_target.id, &None, false);
                target_paths.insert(target, result.0);
            }
        }
    }

    println!("targ paths {}", target_paths.len());

    let mut min_cost = 0;
    let mut returned_val: Option<(NodeId, NodeId, PathedNode)> = None; //source, target, path

    for source_id in sources.iter() {
        let source_path = source_paths.get(source_id).unwrap().as_ref().unwrap();
        for target_id in targets.iter() {
            let target_path = target_paths.get(target_id).unwrap().as_ref().unwrap();
            let path = router.time_expanded_dijkstra(*source_id, *target_id);
            if let Some(transit_path) = path {
                let new_cost = transit_path.cost_from_start
                    + source_path.distance_from_start
                    + target_path.distance_from_start;
                if new_cost > min_cost {
                    min_cost = new_cost;
                    returned_val = Some((*source_id, *target_id, transit_path));
                }
            }
        }
    }
    returned_val
}
