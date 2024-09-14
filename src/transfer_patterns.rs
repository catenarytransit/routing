use crate::NodeType;
//THE FINAL BOSS
use crate::{road_dijkstras::*, transit_dijkstras::*, transit_network::*};
use geo::algorithm::haversine_distance::*;
use geo::point;
use geo::Point;
use rstar::*;
use std::cmp::Reverse;
use std::collections::hash_map::Entry;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::Arc;
use std::time::Instant;
use crate::coord_int_convert::*;

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
            node_type: NodeType::Untyped,
            station_id: tail.station_id,
            time: None,
            trip_id: 0,
            lat: tail.lat,
            lon: tail.lon,
        };
        time_independent_nodes.insert(ti_tail);
        for (head, cost) in edge {
            let ti_head = NodeId {
                node_type: NodeType::Untyped,
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
        day_of_week:"".to_string(),
        transfer_buffer: router.graph.transfer_buffer,
        nodes: time_independent_nodes,
        edges: time_independent_edges,
        station_mapping: HashMap::from([]),
        nodes_per_station: HashMap::from([]),
        trip_mapping: HashMap::from([]),
    };

    let mut time_independent_router = TransitDijkstra::new(&time_independent_graph);
    time_independent_router.set_cost_upper_bound(cost_limit);

    let mut hub_list: HashMap<NodeId, u16> = HashMap::new();

    for _ in 0..random_samples {
        let current_node = time_independent_router.get_random_node_id();
        let visited_nodes = time_independent_router
            .time_expanded_dijkstra(current_node, None, None, None)
            .1;
        for (node, _) in visited_nodes.iter() {
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
    router: &TransitDijkstra,
    hubs: Option<&HashSet<i64>>,
    start_time: Option<u64>
) -> HashMap<(NodeId, NodeId), Vec<NodeId>> {
    if hubs.is_none() {println!("a");}
    let source_transfer_nodes: Option<Vec<NodeId>> = Some(
        router
            .graph
            .nodes_per_station
            .get(&source_station_id)
            .unwrap()
            .iter()
            .filter(|(_, node)| {
                node.node_type == NodeType::Transfer
                || (hubs.is_none() && node.node_type == NodeType::Arrival)
                && node.time >= start_time
            })
            //must check for transfer nodes, but checking for arrival nodes may improve query time at expense of longer precompute
            .map(|(_, node)| *node)
            .collect(),
    );
    if hubs.is_none() {println!("b");}

    let visited_nodes = router
        .time_expanded_dijkstra(None, source_transfer_nodes, None, hubs)
        .1;

    if hubs.is_none() {println!("c");}

    let mut arrival_nodes: Vec<(NodeId, Vec<NodeId>, u64)> = visited_nodes
        .iter()
        .filter(|(node, _)| node.node_type == NodeType::Arrival)
        .map(|(node, pathed_node)| {
            let (path, cost) = pathed_node.clone().get_path();
            (*node, path, cost)
        })
        .collect();

    arrival_loop(&mut arrival_nodes);

    if hubs.is_none() {println!("d");}

    /*for (target, path, _) in arrival_nodes.iter() {
        let mut transfers = Vec::new();
        transfers.push(*target);
        let mut previous_node: NodeId = *target;
        for &node in path {
            if previous_node.node_type == NodeType::Departure
                && node.node_type == NodeType::Transfer
            {
                transfers.push(node);
            }
            previous_node = node;
        }

        transfers.reverse();

        transfer_patterns.insert((*transfers.first().unwrap(), *target), transfers);
    }*/
  
    use std::sync::Mutex;
    use std::thread;

    let total_transfer_patterns = Arc::new(Mutex::new(HashMap::new()));
    let thread_num = 8;
    let source_chunk_len = arrival_nodes.len();
    let threaded_sources = Arc::new(arrival_nodes.clone());
    let mut handles = vec![];
    let now = Instant::now();

    for x in 1..thread_num {
        let source = Arc::clone(&threaded_sources);
        let transfer_patterns = Arc::clone(&total_transfer_patterns);
        let handle = thread::spawn(move || {
            let src = source;
            for i in ((x - 1) * (source_chunk_len / (thread_num - 1)))
                ..(x * source_chunk_len / (thread_num - 1))
            {
                let (target, path, _) = src.get(i).unwrap();
                let mut transfers = Vec::new();
                transfers.push(*target);
                let mut previous_node: NodeId = *target;
                for &node in path {
                    if previous_node.node_type == NodeType::Departure
                        && node.node_type == NodeType::Transfer
                    {
                        transfers.push(node);
                    }
                    previous_node = node;
                }

                transfers.reverse();

                transfer_patterns.lock().unwrap().insert((*transfers.first().unwrap(), *target), transfers);
            }
        });

        handles.push(handle);
    }    

    for handle in handles {
        handle.join().unwrap();
    }

    if hubs.is_none(){println!("elapsed {:?}", now.elapsed());}

    let lock = Arc::try_unwrap(total_transfer_patterns).expect("failed to move out of arc");
    lock.into_inner().expect("mutex could not be locked")
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
    source_lon: f64,
    source_lat: f64,
    target_lon: f64,
    target_lat: f64,
) -> (Point, Point) {
    let source = point!(x: source_lon, y:source_lat);
    let target = point!(x: target_lon, y:target_lat);
    (source, target)
}

pub fn query_graph_construction_from_geodesic_points(
    router: &mut TransitDijkstra,
    source: Point,
    target: Point,
    start_time: u64,
    preset_distance: f64, //in meters
) -> (Vec<NodeId>, Vec<NodeId>, HashMap<NodeId, Vec<NodeId>>) {
    //source nodes, target nodes, edges

    //compute sets of N(source) and N(target) of stations N= near
    let sources:Vec<_> =
    router
    .graph.nodes
    .iter()
    .filter(|node| {
        let node_coord = point!(x: node.lon as f64 / f64::powi(10.0, 14), y: node.lat as f64 / f64::powi(10.0, 14));
        source.haversine_distance(&node_coord) <= preset_distance
            && node.time >= Some(start_time)
            && node.time <= Some(start_time + 7200) //2 hr arbitrary buffer for testing purposes
    })
    .copied()
    .collect();

    let earliest_departure = sources.iter().min_by_key(|a| a.time).unwrap().time;

    let targets:Vec<_> =
    router
    .graph.nodes
    .iter()
    .filter(|node| {
        let node_coord = point!(x: node.lon as f64 / f64::powi(10.0, 14), y: node.lat as f64 / f64::powi(10.0, 14));
        target.haversine_distance(&node_coord) <= preset_distance
        && node.time >= earliest_departure
        && node.time <= Some(earliest_departure.unwrap() + 3600) //1 hr arbitrary buffer for testing purposes
    })
    .copied()
    .collect();

    println!("Possible ending nodes count: {}", targets.len());

    //get hubs of important stations I(hubs)
    let hubs = hub_selection(router, 10000, 54000); //cost limit at 15 hours, arbitrary

    let thread_num = 9;

    //let mut time_tracker_for_multithreading_test = Vec::new();
    //for _ in 1..50 {
    //let find_transfer_patterns = Instant::now();
    use std::sync::Mutex;
    use std::thread;

    let total_transfer_patterns = Arc::new(Mutex::new(HashMap::new()));

    //precompute local TP from N(source) to first hub (this min hub is access station)
    //attempt at multithreading to go faster
    router.node_deactivator(&hubs);

    let source_chunk_len = sources.len();
    let threaded_sources = Arc::new(sources.clone());
    let arc_router = Arc::new(router.clone());
    let threaded_hubs = Arc::new(hubs.clone());
    let mut handles = vec![];

    for x in 1..thread_num {
        let source = Arc::clone(&threaded_sources);
        let transfer_patterns = Arc::clone(&total_transfer_patterns);
        let router = Arc::clone(&arc_router);
        let hub_list = Arc::clone(&threaded_hubs);
        let handle = thread::spawn(move || {
            let src = source;
            for i in ((x - 1) * (source_chunk_len / (thread_num - 1)))
                ..(x * source_chunk_len / (thread_num - 1))
            {
                let source_id = src.get(i).unwrap();
                let l_tps = num_transfer_patterns_from_source(
                    source_id.station_id,
                    &router,
                    Some(&hub_list),
                    Some(start_time)
                );
                
                let mut ttp = transfer_patterns.lock().unwrap();
                ttp.extend(l_tps.into_iter());
            }
        });

        handles.push(handle);
    }    

    for handle in handles {
        handle.join().unwrap();
    }
    
    let mut tps = total_transfer_patterns.lock().unwrap();
    let a: Vec<_>= tps.iter().map(|((_, t), _)| t.station_id).collect();
    let used_hubs: Vec<_> = hubs.into_iter().filter(|n| a.contains(n)).collect();

    //global transfer patterns from I(hubs) to to N(target())
    let hub_chunk_len = used_hubs.len();
    
    println!("num hubs used {}", hub_chunk_len);

    /*let arc_router = Arc::new(router.clone());
    let threaded_hubs = Arc::new(used_hubs.clone());
    let mut handles = vec![];

    for x in 1..thread_num {
        let transfer_patterns = Arc::clone(&total_transfer_patterns);
        let router = Arc::clone(&arc_router);
        let hub_list = Arc::clone(&threaded_hubs);
        let handle = thread::spawn(move || {
            let src = hub_list;
            for i in
                (x - 1) * (hub_chunk_len / (thread_num - 1))..(x * hub_chunk_len / (thread_num - 1))
            {
                let hub_id = src.get(i).unwrap();
                let g_tps = num_transfer_patterns_from_source(*hub_id, &router, None);

                let mut ttp = transfer_patterns.lock().unwrap();
                ttp.extend(g_tps.into_iter());
            }
        });
        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap();
    }*/

    for hub in used_hubs.iter() {
        println!("hub");
        let g_tps = num_transfer_patterns_from_source(*hub, router, None, Some(start_time));
        //tps.extend(g_tps.into_iter().filter(|((_,  t), _)| targets.contains(t)));
        tps.extend(g_tps.into_iter());
    }

    //time_tracker_for_multithreading_test.push(find_transfer_patterns.elapsed().as_secs_f32());

    //println!(
    //    "avg time {:?} vs thread num {}",
    //    time_tracker_for_multithreading_test.iter().sum::<f32>() / time_tracker_for_multithreading_test.len() as f32,
    //    thread_num
    //);

    println!("hubs raw tps num {}", tps.len());

    let paths = tps
        .iter()
        .filter(|((source, target), _)| sources.contains(source) || targets.contains(target))
        //.map(|(_, path)| path)
        .collect::<Vec<_>>();

    println!("paths num {}", paths.len());
    //}

    /*
    let mut total_transfer_patterns = HashMap::new();
    for hub in hubs.iter() {
        let tps = num_transfer_patterns_from_source(*hub, router, None);
        total_transfer_patterns.extend(tps.into_iter());
    }

    router.node_deactivator(&hubs);

    let hubs = Some(hubs);
    for source in sources.iter() {
        let tps = num_transfer_patterns_from_source(source.station_id, router, hubs.as_ref());
        total_transfer_patterns.extend(tps.into_iter());
    }

    println!(
        "tps length {}, time for tps {:?}",
        total_transfer_patterns.len(),
        find_transfer_patterns.elapsed()
    );

    let paths = total_transfer_patterns
        .iter()
        .filter(|((source, target), _)| sources.contains(source) && targets.contains(target))
        .map(|(_, path)| path)
        .collect::<Vec<_>>();
    */
    let mut raw_edges = HashMap::new();

    for path in paths.iter() {
        let mut prev = None;
        for node in path.1.iter() {
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
    }

   // let mut sources: Vec<_> = paths.iter().map(|((s, _), _)| *s).collect();
    //let mut targets: Vec<_> = paths.iter().map(|((_, t), _)| *t).collect();

    //sources.dedup();
   // targets.dedup();

    (sources, targets, raw_edges)
}

pub fn query_graph_search(
    roads: &RoadNetwork,
    connections: DirectConnections,
    edges: HashMap<NodeId, Vec<NodeId>>,
    source: Point,
    target: Point,
    source_target_vecs: (Vec<NodeId>, Vec<NodeId>),
    preset_distance: f64,
) -> Option<(NodeId, NodeId, PathedNode)> {
    let mut source_paths: HashMap<&NodeId, RoadPathedNode> = HashMap::new();

    let road_node_tree = RTree::bulk_load(roads.nodes.values().map(|n| int_to_coord(n.lon, n.lat)).collect());

    println!("Made rtree");

    let mut graph = RoadDijkstra::new(roads);

    println!("Made graph of road dijkstras");

    graph.set_cost_upper_bound((preset_distance / (4.0 * 5.0 / 18.0)) as u64);

    if let Some(start_road_node) = road_node_tree.nearest_neighbor(&(
        source.0.x,
        source.0.y,
    )) {
        for source in source_target_vecs.0.iter() {
            //println!("node {:?}", source.lon);
            if let Some(station_sought) = road_node_tree.nearest_neighbor(&int_to_coord(source.lon, source.lat))
            {
                //println!("neighbor {:?}", station_sought.0);
                //println!("{:?} versus {:?}", start_road_node, station_sought);
                let road_source = *roads
                    .nodes_by_coords
                    .get(&coord_to_int(start_road_node.0, start_road_node.1))
                    .unwrap();
                let station = *roads
                    .nodes_by_coords
                    .get(&coord_to_int(station_sought.0, station_sought.1))
                    .unwrap();
                if let Some(result) = graph.dijkstra(road_source, station) {
                   // println!("{:?} to {:?}", source, result);
                    source_paths.insert(source, result);
                }
            }
        }
    }

    println!("source paths {}", source_paths.len());

    let mut target_paths: HashMap<&NodeId, RoadPathedNode> = HashMap::new();

    if let Some(end_road_node) = road_node_tree.nearest_neighbor(&(
        source.0.x,
        source.0.y,
    )) {
        for target in source_target_vecs.0.iter() {
            //println!("node {:?}", target.lon);
            if let Some(station_sought) = road_node_tree.nearest_neighbor(&int_to_coord(target.lon, target.lat))
            {
                let road_target = *roads
                    .nodes_by_coords
                    .get(&coord_to_int(end_road_node.0, end_road_node.1))
                    .unwrap();
                let station = *roads
                    .nodes_by_coords
                    .get(&coord_to_int(station_sought.0, station_sought.1))
                    .unwrap();
                if let Some(result) = graph.dijkstra(station, road_target) {
                    target_paths.insert(target, result);
                }
            }
        }
    }

    println!("targ paths {}", target_paths.len());

    let mut min_cost = 0;
    let mut router = TDDijkstra::new(connections, edges);
    let mut returned_val: Option<(NodeId, NodeId, PathedNode)> = None; //source, target, path

    for source_id in source_target_vecs.0.iter() {
        let source_path = source_paths.get(source_id).unwrap();
        for target_id in source_target_vecs.1.iter() {
            let target_path = target_paths.get(target_id).unwrap();
            let path = router.time_dependent_dijkstra(*source_id, *target_id);
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
