use crate::NodeType;
//THE FINAL BOSS
use crate::coord_int_convert::*;
use crate::RoadNetwork;
use crate::{road_dijkstras::*, transit_dijkstras::*, transit_network::*};
use geo::{point, Distance, Haversine, Point};
use rstar::*;
use serde::{Deserialize, Serialize};
use std::collections::hash_map::Entry;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::Arc;

//only calculate global time expanded dijkstra from hubs (important stations) to save complexity
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
                    let mut map: HashMap<NodeId, u64> = HashMap::new();
                    map.insert(ti_head, *cost);
                    map
                });
        }
    }

    let time_independent_graph = TimeExpandedGraph {
        //transfer_buffer: router.graph.transfer_buffer,
        nodes: time_independent_nodes,
        edges: time_independent_edges,
        station_mapping: HashMap::from([]),
        station_info: HashMap::from([]),
    };

    let mut time_independent_router = TransitDijkstra::new(&time_independent_graph);
    time_independent_router.set_cost_upper_bound(cost_limit);

    let mut hub_list: HashMap<NodeId, u16> = HashMap::new();

    for _ in 0..random_samples {
        let current_node = vec![time_independent_router.get_random_node_id().unwrap()];
        let visited_nodes = time_independent_router
            .time_expanded_dijkstra(current_node, None, None)
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
// Return the transfer patterns & numbers between each station pair.
pub fn num_transfer_patterns_from_source(
    source_station_id: i64,
    router: &TransitDijkstra,
    hubs: Option<&HashSet<i64>>,
    start_time: Option<u64>,
) -> HashMap<(NodeId, NodeId), Vec<NodeId>> {
    let source_transfer_nodes: Vec<NodeId> = 
        router
            .graph
            .station_info
            .get(&source_station_id)
            .unwrap()
            .1
            .iter()
            .filter(|(_, node)| {
                node.node_type == NodeType::Transfer
                    || (hubs.is_none() && node.node_type == NodeType::Arrival)
                        && node.time >= start_time
            })
            //must check for transfer nodes, but checking for arrival nodes may improve query time at expense of longer precompute
            .map(|(_, node)| *node)
            .collect();
    println!("a\t");
    let visited_nodes = router
        .time_expanded_dijkstra(source_transfer_nodes, None, hubs)
        .1;
    println!("b\t");
    let mut arrival_nodes: Vec<(NodeId, Vec<NodeId>, u64)> = visited_nodes
        .iter()
        .filter(|(node, _)| node.node_type == NodeType::Arrival)
        .map(|(node, pathed_node)| {
            let (path, cost) = pathed_node.clone().get_path();
            (*node, path, cost)
        })
        .collect();
    println!("c\t");
    arrival_loop(&mut arrival_nodes);
    println!("d\t");

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

                transfer_patterns
                    .lock()
                    .unwrap()
                    .insert((*transfers.first().unwrap(), *target), transfers);
            }
        });

        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap();
    }
    println!("e\t");

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

#[derive(Serialize, Deserialize, Debug)]
pub struct QueryGraphItem {
    source: Point,
    target: Point,
    edges: HashMap<NodeId, Vec<NodeId>>,
    source_stations: Vec<StationInfo>,
    target_stations: Vec<StationInfo>,
    source_nodes: Vec<NodeId>,
    target_nodes: Vec<NodeId>,
}

pub fn query_graph_construction_from_geodesic_points(
    router: &mut TransitDijkstra,
    source: Point,
    target: Point,
    start_time: u64,
    preset_distance: f64, //in meters
) -> QueryGraphItem {
    //source nodes, target nodes, edges

    //compute sets of N(source) and N(target) of stations N= near
    let (source_ids, (source_stations, nodes_per_source)): (Vec<i64>, (Vec<_>, Vec<_>))=
    router
    .graph.station_info
    .clone()
    .into_iter()
    .filter(|(_, station)| {
        let node_coord = point!(x: station.0.lon as f64 / f64::powi(10.0, 14), y: station.0.lat as f64 / f64::powi(10.0, 14));
        Haversine::distance(source, node_coord) <= preset_distance
    })
    .unzip();

    println!("Possible start nodes count: {}", source_ids.len());

    //let earliest_departure = sources.iter().min_by_key(|a| a.time).unwrap().time;

    let (target_ids, (target_stations, nodes_per_target)): (Vec<i64>, (Vec<_>, Vec<_>))=
    router
    .graph.station_info
    .clone()
    .into_iter()
    .filter(|(_, station)| {
        let node_coord = point!(x: station.0.lon as f64 / f64::powi(10.0, 14), y: station.0.lat as f64 / f64::powi(10.0, 14));
        Haversine::distance(target, node_coord) <= preset_distance
    })
    .unzip();

    println!("Possible end nodes count: {}", target_ids.len());

    //get hubs of important stations I(hubs)
    let hubs = hub_selection(router, 10000, 36000); //cost limit at 10 hours, arbitrary

    use std::sync::Mutex;
    use std::thread;
    let thread_num = 10;
    let total_transfer_patterns = Arc::new(Mutex::new(HashMap::new()));

    //precompute local TP from N(source) to first hub (this min hub is access station)

    let source_chunk_len = source_ids.len();
    let threaded_sources = Arc::new(source_ids.clone());
    let arc_router = Arc::new(router.clone());
    let threaded_hubs = Arc::new(hubs.clone());
    let mut handles = vec![];

    println!("local tps");
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
                    *source_id,
                    &router,
                    Some(&hub_list),
                    Some(start_time),
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
    println!("source to hub tps num {}", tps.len());
    let used_hubs: Vec<_> = hubs.into_iter().filter(|n| source_ids.contains(n)).collect();

    //global transfer patterns from I(hubs) to to N(target())
    println!("num hubs used {:?}", used_hubs.len());

    for hub in used_hubs.iter() {
        let g_tps = num_transfer_patterns_from_source(*hub, router, None, Some(start_time));
        tps.extend(g_tps.into_iter());
    }

    println!("plus hubs to target tps num {}", tps.len());

    let source_nodes: Vec<_> = nodes_per_source
        .into_iter()
        .flat_map(|x| {
            x.into_iter()
                .unzip::<u64, NodeId, Vec<u64>, Vec<NodeId>>()
                .1
        })
        .filter(|node| node.time >= Some(start_time) && node.time <= Some(start_time + 3600))
        .collect();

    let earliest_departure = source_nodes.iter().min_by_key(|a| a.time).unwrap().time;

    let target_nodes: Vec<_> = nodes_per_target
        .into_iter()
        .flat_map(|x| {
            x.into_iter()
                .unzip::<u64, NodeId, Vec<u64>, Vec<NodeId>>()
                .1
        })
        .filter(|node| {
            node.time >= earliest_departure && node.time <= Some(earliest_departure.unwrap() + 3600)
        })
        .collect();

    let paths = tps
        .iter()
        .filter(|((source, target), _)| {
            source_nodes.contains(source) || target_nodes.contains(target)
        })
        .map(|(_, path)| path)
        .collect::<Vec<_>>();

    println!("paths num {}", paths.len());

    //}

    let mut edges = HashMap::new(); //tail, heads

    for path in paths.iter() {
        let mut prev = None;
        for node in path.iter() {
            if let Some(prev) = prev {
                match edges.entry(prev) {
                    Entry::Occupied(mut o) => {
                        let heads: &mut Vec<NodeId> = o.get_mut();
                        heads.push(*node);
                    }
                    Entry::Vacant(v) => {
                        let heads = Vec::from([*node]);
                        v.insert(heads);
                    }
                }
            }
            prev = Some(*node);
        }
    }

    QueryGraphItem {
        source,
        target,
        edges,
        source_stations,
        target_stations,
        source_nodes,
        target_nodes,
    }
}

pub fn query_graph_search(
    roads: &RoadNetwork,
    connections: DirectConnections,
    query_info: QueryGraphItem,
) -> Option<(NodeId, NodeId, PathedNode)> {
    let mut source_paths: HashMap<i64, RoadPathedNode> = HashMap::new();

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

                if let Some(result) = graph.dijkstra(road_source, station) {
                    source_paths.insert(source.id, result);
                }
            }
        }
    }

    println!("source paths {:?}", source_paths.keys());

    let mut target_paths: HashMap<i64, RoadPathedNode> = HashMap::new();

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
                if let Some(result) = graph.dijkstra(station, road_target) {
                    target_paths.insert(target.id, result);
                }
            }
        }
    }

    println!("targ paths {:?}", target_paths.keys());

    let mut min_cost = 0;
    let mut router = TDDijkstra::new(connections, query_info.edges);
    let mut returned_val: Option<(NodeId, NodeId, PathedNode)> = None; //source, target, path

    for source_id in query_info.source_nodes.iter() {
        if let Some(source_path) = source_paths.get(&source_id.station_id){
            for target_id in query_info.target_nodes.iter() {
                if let Some(target_path) = target_paths.get(&target_id.station_id){
                    let path = router.time_dependent_dijkstra(*source_id, *target_id);
                    if let Some(transit_path) = path {
                        println!("aaugh");
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
        }
    }

    returned_val
}
