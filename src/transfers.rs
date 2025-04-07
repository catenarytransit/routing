use crate::NodeType;
//use crate::coord_int_convert::*;
//use crate::RoadNetwork;
use crate::{transit_dijkstras::*, transit_network::*};
//use crate::road_dijkstras::*;
use geo::{point, Distance, Haversine, Point};
//use rstar::*;
use serde::{Deserialize, Serialize};
use std::collections::hash_map::Entry;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Instant;

#[derive(Serialize, Deserialize, Debug)]
pub struct QueryGraph {
    source: Point,
    target: Point,
    pub edges: HashMap<NodeId, HashSet<NodeId>>,
    source_stations: HashSet<Station>,
    target_stations: HashSet<Station>,
    hubs: HashSet<i64>,
    source_nodes: HashSet<NodeId>,
    target_nodes: HashSet<NodeId>,
    station_map: HashMap<String, Station>,
}

pub fn query_graph_construction(
    router: &mut TransitDijkstra,
    maps: &NumberNameMaps,
    paths: &mut HashMap<NodeId, PathedNode>,
    source: Point,
    target: Point,
    start_time: u32,
    preset_distance: f64, //in meters
) -> QueryGraph {
    let now = Instant::now();

    //compute sets of N(source) and N(target) of stations N= near
    let (source_stations, source_nodes): (HashSet<_>, HashSet<_>) =
        stations_near_point(router, source, preset_distance, start_time, None);

    let end_time = Some(3600 * 3); //3 hours
    let (target_stations, target_nodes): (HashSet<_>, HashSet<_>) =
        stations_near_point(router, target, preset_distance, start_time, end_time);

    let hub_cost_limit = 86400; //24 hour searchspace

    //get hubs of important stations I(hubs)
    let hubs = hub_selection(router, maps, 50000, hub_cost_limit); //cost limit at 10 hours, arbitrary

    println!("hubs: {:?}, t {:?}", &hubs, now.elapsed());

    let mut tps = Vec::new();

    for station in source_stations.iter() {
        let now = Instant::now();
        let (l_tps, n_now) = transfers_from_source(
            station.id,
            router,
            Some(&hubs),
            paths,
            None,
            Some(start_time),
        );
        println!(
            "local tp {:?} or immediate {:?}",
            now.elapsed(),
            n_now.elapsed()
        );

        let now = Instant::now();
        tps.extend(l_tps.lock().unwrap().drain(..));
        println!("extending local {:?}", now.elapsed());
    }

    println!("currently, tps are\n{:?}", tps);

    let now = Instant::now();
    //let reached: Vec<_> = tps.iter().map(|t| t.last().unwrap().station_id).collect();
    let used_hubs: Vec<_> = hubs.iter().collect(); //hubs.iter().filter(|n| reached.contains(n)).collect();

    let target_ids = target_stations.iter().map(|id| id.id).collect();

    //global transfers from I(hubs) to to N(target())
    println!("num hubs used {:?}, t {:?}", used_hubs, now.elapsed());

    for hub in used_hubs.iter() {
        let now = Instant::now();
        let (g_tps, n_now) = transfers_from_source(
            **hub,
            router,
            None,
            paths,
            Some(&target_ids),
            Some(start_time),
        );
        println!(
            "ran tp for hubs {:?} vs immediate {:?}",
            now.elapsed(),
            n_now.elapsed()
        );

        let now = Instant::now();
        tps.extend(g_tps.lock().unwrap().drain(..));
        println!("extending hubs {:?}", now.elapsed());
    }

    let now = Instant::now();

    let paths = tps
        .iter()
        .filter(|v| {
            source_nodes.contains(v.first().unwrap()) || target_nodes.contains(v.last().unwrap())
        })
        .collect::<Vec<_>>();

    println!("paths {} and {:?}", paths.len(), now.elapsed());

    let now = Instant::now();

    let mut edges = HashMap::new(); //tail, heads

    for path in paths.iter() {
        let mut prev = None;
        for node in path.iter() {
            if let Some(prev) = prev {
                match edges.entry(prev) {
                    Entry::Occupied(mut o) => {
                        let heads: &mut HashSet<NodeId> = o.get_mut();
                        heads.insert(*node);
                    }
                    Entry::Vacant(v) => {
                        let heads = HashSet::from([*node]);
                        v.insert(heads);
                    }
                }
            }
            prev = Some(*node);
        }
    }

    println!("collecting paths {:?}", now.elapsed());

    let station_map = maps.station_map.clone();

    QueryGraph {
        source,
        target,
        edges,
        source_stations,
        target_stations,
        hubs,
        source_nodes,
        target_nodes,
        station_map: station_map.unwrap(),
    }
}

pub fn stations_near_point(
    router: &TransitDijkstra,
    point: Point,
    preset_distance: f64,
    start_time: u32,
    end_time: Option<u32>,
) -> (HashSet<Station>, HashSet<NodeId>) {
    let now = Instant::now();
    let (point_stations, nodes_per_point): (HashSet<_>, Vec<_>)=
        router
        .graph
        .station_info
        .as_ref()
        .unwrap()
        .clone()
        .into_iter()
        .filter(|(station, _)| {
            let node_coord = point!(x: station.lon as f64 / f64::powi(10.0, 14), y: station.lat as f64 / f64::powi(10.0, 14));
            Haversine::distance(point, node_coord) <= preset_distance
        })
        .unzip();

    let point_nodes: HashSet<NodeId> = nodes_per_point
        .into_iter()
        .flat_map(|x| {
            x.into_iter()
                .unzip::<u32, NodeId, Vec<u32>, Vec<NodeId>>()
                .1
        })
        .filter(|node| {
            node.time >= Some(start_time) && node.time <= Some(start_time + end_time.unwrap_or(0))
        })
        .collect();

    println!(
        "Possible nodes count: {}, t {:?}",
        point_stations.len(),
        now.elapsed()
    );

    (point_stations, point_nodes)
}

//only calculate global time expanded dijkstra from hubs (important stations) to save complexity
//picks important hubs if they are more often visted from Dijkstras-until-all-nodes-settled
pub fn hub_selection(
    router: &TransitDijkstra,
    maps: &NumberNameMaps,
    random_samples: u32,
    cost_limit: u32,
) -> HashSet<i64> {
    //station ids
    let num_stations = 1.max((maps.station_map.as_ref().unwrap().len() as u32) / 100);
    let mut selected_hubs: HashSet<i64> = HashSet::new();

    let mut time_independent_edges: HashMap<NodeId, HashMap<NodeId, u32>> = HashMap::new();

    let mut time_independent_nodes = HashSet::new();

    for (tail, edge) in router.graph.edges.iter() {
        let ti_tail = NodeId {
            node_type: NodeType::Untyped,
            station_id: tail.station_id,
            time: None,
            trip_id: 0,
        };
        time_independent_nodes.insert(ti_tail);
        for (head, cost) in edge {
            let ti_head = NodeId {
                node_type: NodeType::Untyped,
                station_id: head.station_id,
                time: None,
                trip_id: 0,
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
                    let mut map: HashMap<NodeId, u32> = HashMap::new();
                    map.insert(ti_head, *cost);
                    map
                });
        }
    }

    let time_independent_graph = TimeExpandedGraph {
        //transfer_buffer: router.graph.transfer_buffer,
        nodes: time_independent_nodes,
        edges: time_independent_edges,
        station_info: None,
    };

    let (mut time_independent_router, mut paths) = TransitDijkstra::new(time_independent_graph);
    time_independent_router.set_cost_upper_bound(cost_limit);

    let mut hub_list: HashMap<NodeId, u16> = HashMap::new();

    for _ in 0..random_samples {
        let current_node = vec![time_independent_router.get_random_node_id().unwrap()];
        let visited_nodes =
            time_independent_router.time_expanded_dijkstra(current_node, None, &mut paths);
        for node in visited_nodes.iter() {
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

    time_independent_router.set_cost_upper_bound(u32::MAX); //reset cost upper bound to max

    selected_hubs
}

// Precompute transfers from a given station to all other stations.
// Return the transfers & numbers between each station pair.
pub fn transfers_from_source(
    source_station_id: i64,
    router: &TransitDijkstra,
    hubs: Option<&HashSet<i64>>,
    paths: &mut HashMap<NodeId, PathedNode>,
    targets: Option<&HashSet<i64>>,
    start_time: Option<u32>,
) -> (Mutex<Vec<Vec<NodeId>>>, Instant) {
    println!("start tp calc \t");
    let now = Instant::now();
    let source_transfer_nodes: Vec<NodeId> = router
        .graph
        .nodes
        .iter()
        .filter(|node| {
            node.station_id == source_station_id
                && (node.node_type == NodeType::Transfer
                    || (hubs.is_none() && node.node_type == NodeType::Arrival))
                && node.time >= start_time
        })
        //must check for transfer nodes, but checking for arrival nodes may improve query time at expense of longer precompute
        .copied()
        .collect();
    println!("found sources {:?}", now.elapsed());
    println!("source are\n{:?}", source_transfer_nodes);
    let now = Instant::now();

    let visited_nodes = router.time_expanded_dijkstra(source_transfer_nodes, hubs, paths);

    println!("visited nodes {:?}", now.elapsed());
    println!("visted are\n{:?}", visited_nodes);

    let now = Instant::now();

    let mut arrival_nodes: Vec<(NodeId, Vec<NodeId>, u32)> = visited_nodes
        .into_iter()
        .filter(|node| node.node_type == NodeType::Arrival)
        .map(|node| {
            let (mut path, cost) = PathedNode::get_path(node, paths);
            //path.reverse();
            if hubs.is_some() {
                let len = path
                    .iter()
                    .position(|node| hubs.as_ref().unwrap().contains(&node.station_id))
                    .unwrap_or(path.len());
                let u: Vec<_> = path.drain(len..).collect();
                let mut iter = u.chunk_by(|a, b| a.station_id == b.station_id);
                let add_back_node = iter.next().unwrap_or(&[]);
                //println!("{:?}", add_back_node);
                path.extend(add_back_node);
                //if len < path.len(){println!("hub t {len}")};
            }
            if targets.is_some() {
                let len = path
                    .iter()
                    //err cant find lines per station
                    //chunk this and find the last node in this sequence for the target found?
                    .position(|node| targets.as_ref().unwrap().contains(&node.station_id))
                    .unwrap_or(0);
                if len != 0 {
                    let u: Vec<_> = path.drain(len..).collect();
                    //println!("{:?}", u);
                    let mut iter = u.chunk_by(|a, b| a.station_id == b.station_id);
                    let add_back_node = iter.next().unwrap_or(&[]);
                    //println!("{:?}", add_back_node);
                    path.extend(add_back_node);
                } else {
                    path.clear();
                }
                //if len > 0 {println!("targ t {len}")};
            }
            (node, path, cost)
        })
        .collect();
    arrival_nodes.retain(|(_, path, _)| !path.is_empty());
    println!("filtered arrivals {:?}", now.elapsed());
    let now = Instant::now();

    arrival_loop(&mut arrival_nodes);
    println!("arrival loop {:?}", now.elapsed());
    let now = Instant::now();

    let total_transfers = Arc::new(Mutex::new(Vec::new()));
    let thread_num = 4;
    let source_chunk_len = arrival_nodes.len();
    let threaded_sources = Arc::new(arrival_nodes.clone());
    let mut handles = vec![];

    for x in 1..thread_num {
        let source = Arc::clone(&threaded_sources);
        let transfers = Arc::clone(&total_transfers);
        let handle = thread::spawn(move || {
            let src = source;
            for i in ((x - 1) * (source_chunk_len / (thread_num - 1)))
                ..(x * source_chunk_len / (thread_num - 1))
            {
                let (target, path, _) = src.get(i).unwrap();
                let mut loc_transfers = Vec::new();
                //transfers.push(*target);
                //let mut previous_node: NodeId = *target;
                for &node in path {
                    if
                    //previous_node.node_type == NodeType::Departure
                    //|| previous_node.node_type == NodeType::Transfer &&
                    node.node_type == NodeType::Arrival || node.node_type == NodeType::Transfer {
                        loc_transfers.push(node);
                    }
                    //previous_node = node;
                }

                loc_transfers.push(*target);
                //transfers.reverse();

                transfers.lock().unwrap().push(loc_transfers);
            }
        });

        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap();
    }

    let lock = Arc::try_unwrap(total_transfers).expect("failed to move out of arc");
    println!("found transfers {:?}", now.elapsed());
    let now = Instant::now();
    (lock, now)
}

// Arrival chain algo: For each arrival node, see if cost can be improved
// by simply waiting from an earlier arrival time. (favors less travel time)
pub fn arrival_loop(arrival_nodes: &mut [(NodeId, Vec<NodeId>, u32)]) {
    arrival_nodes.sort_unstable_by(|a, b| a.0.station_id.cmp(&b.0.station_id));
    let time_chunks = arrival_nodes.chunk_by_mut(|a, b| a.0.station_id <= b.0.station_id);
    for chunk in time_chunks {
        chunk.sort_unstable_by(|a, b| a.0.time.cmp(&b.0.time));
        let mut previous_arrival: Option<(NodeId, u32)> = None;
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
    let source = point!(x: source_lon, y:source_lat);
    let target = point!(x: target_lon, y:target_lat);
    (source, target)
}

pub fn query_graph_search(
    //roads: &RoadNetwork,
    connections: DirectConnections,
    query_info: QueryGraph,
    paths: &mut HashMap<NodeId, PathedNode>,
) -> Option<(NodeId, PathedNode)> {
    /*let mut source_paths: HashMap<i64, _> = HashMap::new();

    let road_node_tree = RTree::bulk_load(
        roads
            .nodes
            .values()
            .map(|n| int_to_coord(n.lon, n.lat))
            .collect(),
    );

    println!("Made rtree");

    let mut graph = RoadDijkstra::new(roads);

    println!("Made graph of road dijkstras");

    if let Some(start_road_node) = road_node_tree.nearest_neighbor(&(query_info.source.0.x_y())) {
        for source in query_info.source_stations.iter() {
            if let Some(station_sought) =
                road_node_tree.nearest_neighbor(&int_to_coord(source.lon, source.lat))
            {
                let road_source = *roads
                    .nodes_by_coords
                    .get(&coord_to_int(start_road_node.0, start_road_node.1))
                    .unwrap();
                let station = *roads
                    .nodes_by_coords
                    .get(&coord_to_int(station_sought.0, station_sought.1))
                    .unwrap();

                //if let Some(result) = graph.dijkstra(road_source, station) { //figure this out later, use Valhalla OSM
                let result = 0;
                    source_paths.insert(source.id, result);
                //}
            }
        }
    }

    println!("source paths {:?}", source_paths.keys());

    let mut target_paths: HashMap<i64, _> = HashMap::new();

    if let Some(end_road_node) = road_node_tree.nearest_neighbor(&(query_info.target.0.x_y())) {
        for target in query_info.target_stations.iter() {
            if let Some(station_sought) =
                road_node_tree.nearest_neighbor(&int_to_coord(target.lon, target.lat))
            {
                let road_target = *roads
                    .nodes_by_coords
                    .get(&coord_to_int(end_road_node.0, end_road_node.1))
                    .unwrap();
                let station = *roads
                    .nodes_by_coords
                    .get(&coord_to_int(station_sought.0, station_sought.1))
                    .unwrap();
                //if let Some(result) = graph.dijkstra(station, road_target) { //figure this out later, use Valhalla OSM
                let result = 0;
                    target_paths.insert(target.id, result);
                //}
            }
        }
    }

    println!("targ paths {:?}", target_paths.keys());
    */

    let mut min_cost = 0;
    let mut router = TDDijkstra::new(connections, query_info.edges);

    let mut returned_val: Option<(NodeId, PathedNode)> = None; //source, target, path

    for source_id in query_info.source_nodes.iter() {
        //if let Some(_source_path) = source_paths.get(&source_id.station_id){
        //println!("s {:?}", source_id.station_id);
        //if let Some(_target_path) = target_paths.get(&target_id.station_id){
        //println!("t {:?}", target_id.station_id);
        let path = router.time_dependent_dijkstra(paths, *source_id, &query_info.target_nodes);
        if let Some(target) = path {
            let transit_path = paths.get(&target).unwrap();
            let new_cost = transit_path.cost_from_start;
            //+ source_path.distance_from_start
            //+ target_path.distance_from_start;
            if new_cost > min_cost {
                min_cost = new_cost;
                returned_val = Some((*source_id, transit_path.clone()));
            }
        }
    }
    returned_val
}
