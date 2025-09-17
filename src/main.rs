#![allow(unused)]
// Copyright Chelsea Wen
// Cleaned up somewhat by Kyler Chin
use geo::{point, Point};
use routing::{
    road_dijkstras::*,
    road_network::{
        arc_flags_algo::*, contraction_hierarchies::*, landmark_algo::*, road_graph_construction::*,
    },
    transfers::*,
    transit_dijkstras::*,
    transit_network::*,
};
use serde_json::{Result, Value};
use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::time::Instant;
use tokio::*;

#[cfg(not(target_env = "msvc"))]
use tikv_jemallocator::Jemalloc;

#[cfg(not(target_env = "msvc"))]
#[global_allocator]
static GLOBAL: Jemalloc = Jemalloc;

//converts raw coordinate points into Point structs
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

use clap::Parser;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Number of times to greet
    #[arg(long, default_value_t = false)]
    makequerygraph: bool,
    #[arg(long, default_value_t = true)]
    debugmode: bool,
}

#[tokio::main]
async fn main() {
    std::env::set_var("RUST_BACKTRACE", "1");
    //let path = "bw.pbf";
    //let path = "uci.pbf";

    let mut args = Args::parse();

    if !args.debugmode {
        let preset_distance = 250.0;
        println!("Arguments: {args:#?}");

        let savepath = "results.json";

        println!("generating transit network graph");
        let gtfs = read_from_gtfs_zip("ctt.zip");

        //overhead for cloning these strings is very low, it's just for displaying anyway
        let trips = gtfs.trips.clone();
        let routes = gtfs.routes.clone();
        let stops = gtfs.stops.clone();

        //generate Time Expanded Graph and Direct Connections for this GTFS file
        let (transit_graph, connections, maps) =
            TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 0);
        let (mut router, mut paths) = TransitDijkstra::new(transit_graph);

        println!("{}", router.graph.nodes.len());

        //full routing test
        //see following link, anything but first option (which includes walking between stations, hasnt been implemented yet)
        //https://www.google.com/maps/dir/Bloomfield,+Connecticut+06002/77+Forest+St,+Hartford,+CT+06105/@41.823207,-72.7745391,34082m/data=!3m1!1e3!4m20!4m19!1m5!1m1!1s0x89e7001af40714d7:0xc4be608b22d7e4a8!2m2!1d-72.7197095!2d41.8683576!1m5!1m1!1s0x89e653502e880197:0xc1f0096f7d179457!2m2!1d-72.7005256!2d41.7671825!2m4!4e3!6e0!7e2!8j1727241000!3e3!5i1

        if (args.makequerygraph) {
            //pepperidge farm to harriet beecher stowe center
            let (source, target) = make_points_from_coords(
                41.86829675142084,
                -72.71973332600558,
                41.76726348091365,
                -72.70049435551549,
            );

            let now = Instant::now();
            let graph = query_graph_construction(
                &mut router,
                &maps,
                &mut paths,
                source,
                target,
                18600, //5:10 AM
                preset_distance,
            );

            let output = File::create(savepath).unwrap();
            println!("query graph constructed in {:?}", now.elapsed());
            serde_json::to_writer(output, &graph).unwrap();
        }

        //part 2

        let file = File::open(savepath).ok().unwrap();
        let reader = BufReader::new(file);
        let graph: QueryGraph = serde_json::from_reader(reader).unwrap();

        let run_query = query_graph_search(connections, graph, &mut paths);

        if let Some((s, t, pathed)) = run_query {
            let path = pathed.get_tp(s, &paths, &maps);
            for (node, route) in path.0 {
                println!("{node:?}");
                if let Some(route) = route {
                    println!("via {:?}", routes.get(&route).unwrap().short_name);
                } else {
                    println!("start {}", stops.get(&node.station_id.to_string()).unwrap());
                }
            }
        } else {
            println!("route not found");
        }
    } else {
        let preset_distance = 0.0;
        println!("debug mode");
        let gtfs = read_from_gtfs_zip("test.zip");

        //overhead for cloning these strings is very low, it's just for displaying anyway
        let routes = gtfs.routes.clone();
        let stops = gtfs.stops.clone();

        let (transit_graph, connections, maps) =
            TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 0);
        let (mut router, mut paths) = TransitDijkstra::new(transit_graph);

        let (source, target) = make_points_from_coords(33.0, -117.00001, 33.0, -117.00007);

        let now = Instant::now();
        let graph = query_graph_construction(
            &mut router,
            &maps,
            &mut paths,
            source,
            target,
            21600, //5:10 AM
            preset_distance,
        );

        let output = File::create("test.json").unwrap();
        println!("query graph constructed in {:?}", now.elapsed());
        serde_json::to_writer(output, &graph).unwrap();

        //println!("pathed nodes: {:?}", paths);

        let run_query = query_graph_search(connections, graph, &mut paths);

        if let Some((s, t, pathed)) = run_query {
            println!("pathed: {pathed:?}");
            println!("end {:?}", maps.station_num_to_name(&t.station_id));
            let path = pathed.get_tp(s, &paths, &maps);
            for (node, route) in path.0 {
                println!(
                    "go to {:?} (details {node:?}",
                    maps.station_num_to_name(&node.station_id)
                );
                if let Some(route) = route {
                    println!("via {route:?}");
                } else {
                    println!("start {:?}", maps.station_num_to_name(&node.station_id));
                }
            }
        } else {
            println!("route not found");
        }
    }
}

async fn road_stuff() {
    let path = "saarland.pbf";
    let data = RoadNetwork::read_from_osm_file(path).unwrap();
    let mut roads = RoadNetwork::new(data.0, data.1);
    print!(
        "{} Base Graph Nodes: {}, Edges: {}\t\t",
        path,
        roads.nodes.len(),
        roads.edges.len()
    );
    let now = Instant::now();
    roads = roads.reduce_to_largest_connected_component();
    let mut time = now.elapsed().as_millis() as f32 * 0.001;
    println!(
        "time: {}, reduced map nodes: {}, edges: {}",
        time,
        roads.nodes.len(),
        roads.edges.values().map(|edges| edges.len()).sum::<usize>() / 2
    );

    let mut routing_graph = RoadDijkstra::new(&roads);
    let mut ch_algo = ContractedGraph::new();
    let now = Instant::now();

    routing_graph.reset_all_flags(true);
    routing_graph.set_max_settled_nodes(20);

    ch_algo.compute_random_node_ordering(&mut routing_graph, 1000); //here
    let mut contraction_time = Vec::new();
    let mut shortcut_hg = vec![0, 0, 0, 0, 0];
    let mut edge_diff_hg = vec![0, 0, 0, 0, 0];
    for (&n, _) in ch_algo.ordered_nodes.clone().iter() {
        let now = Instant::now();
        let (num_shortcut, num_edge_diff) = ch_algo.contract_node(n, &mut routing_graph, false);
        time = now.elapsed().as_micros() as f32;
        contraction_time.push(time);

        if num_shortcut == 0 {
            shortcut_hg[0] += 1;
        } else if num_shortcut == 1 {
            shortcut_hg[1] += 1;
        } else if num_shortcut == 2 {
            shortcut_hg[2] += 1;
        } else if num_shortcut == 3 {
            shortcut_hg[3] += 1;
        } else if num_shortcut >= 4 {
            shortcut_hg[4] += 1;
        }

        if num_edge_diff <= -3 {
            edge_diff_hg[0] += 1;
        } else if num_edge_diff == -2 {
            edge_diff_hg[1] += 1;
        } else if (-1..=1).contains(&num_edge_diff) {
            edge_diff_hg[2] += 1;
        } else if num_edge_diff == 2 {
            edge_diff_hg[3] += 1;
        } else if num_edge_diff >= 3 {
            edge_diff_hg[4] += 1;
        }
    }

    println!(
        "average contraction time in micros {}",
        contraction_time.iter().sum::<f32>() / contraction_time.len() as f32
    );

    println!("shortcut histogram {shortcut_hg:?}");

    println!("edge difference histogram {edge_diff_hg:?}");

    routing_graph.reset_all_flags(true);

    let total_shortcuts = ch_algo.ch_precompute(&mut routing_graph);
    time = now.elapsed().as_millis() as f32 * 0.001;
    println!("precomp seconds: {time}");
    println!("total shortcuts: {total_shortcuts}");

    let mut shortest_path_costs = Vec::new();
    let mut query_time = Vec::new();

    for _ in 0..1000 {
        //1000
        let source = routing_graph.get_random_node_id().unwrap();
        let target = routing_graph.get_random_node_id().unwrap();
        let now = Instant::now();
        let result = ContractedGraph::bidirectional_compute(&mut routing_graph, source, target);
        time = now.elapsed().as_millis() as f32 * 0.001;
        query_time.push(time);
        shortest_path_costs.push(result.0);
    }

    println!(
        "average cost mm:ss {}:{} \n\t eg {}, {}, {}",
        shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 / 60,
        shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 % 60,
        shortest_path_costs.get(250).unwrap(),
        shortest_path_costs.get(500).unwrap(),
        shortest_path_costs.get(750).unwrap()
    );
    println!(
        "average query time in seconds {}",
        query_time.iter().sum::<f32>() / query_time.len() as f32
    );

    let mut shortest_path_costs = Vec::new();
    let mut query_time = Vec::new();
    let mut settled_nodes = Vec::new();
    let heuristics = None;

    //let precompute = landmark_heuristic_precompute(&mut routing_graph, 42);
    let arc_flag_thing = ArcFlags::new(49.20, 49.25, 6.95, 7.05); //saar
                                                                  //let arc_flag_thing = ArcFlags::new(33.63, 33.64, -117.84, -117.83); //uci
    arc_flag_thing.arc_flags_precompute(&mut routing_graph);
    time = now.elapsed().as_millis() as f32 * 0.001;
    println!("pre done {time} \n");

    for _ in 0..100 {
        let source = routing_graph.get_random_node_id().unwrap();
        //let target = routing_graph.get_random_node_id().unwrap();
        let target = routing_graph.get_random_node_area_id(49.20, 49.25, 6.95, 7.05); //saar
                                                                                      //let target = routing_graph.get_random_node_area_id(33.63, 33.64, -117.84, -117.83); //uci
                                                                                      //heuristics = a_star_heuristic(&roads, target);
                                                                                      //heuristics = landmark_heuristic(&precompute, &routing_graph, target);
        let now = Instant::now();
        let result = routing_graph.dijkstra(source, target, &heuristics, true);
        time = now.elapsed().as_millis() as f32 * 0.001;
        query_time.push(time);

        if let Some(cost) = result.0 {
            shortest_path_costs.push(cost.get_path().1);
        } else {
            shortest_path_costs.push(0);
        }
        settled_nodes.push(routing_graph.visited_nodes.len() as u64);
    }

    println!(
        "average cost in minutes {}",
        shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 / 60
    );
    println!(
        "average query time in seconds {}",
        query_time.iter().sum::<f32>() / query_time.len() as f32
    );
    println!(
        "average settle node number {}",
        settled_nodes.iter().sum::<u64>() / settled_nodes.len() as u64
    );

    let _node0 = Node {
        id: 0,
        lat: 490000000,
        lon: 65000000,
    };
    let node1 = Node {
        id: 1,
        lat: 491000000,
        lon: 65100000,
    };
    let node2 = Node {
        id: 2,
        lat: 495000000,
        lon: 70000000,
    };
    let node3 = Node {
        id: 3,
        lat: 493500000,
        lon: 71250000,
    };
    let node4 = Node {
        id: 4,
        lat: 492500000,
        lon: 72500000,
    };
    let node5 = Node {
        id: 5,
        lat: 497500000,
        lon: 72500000,
    };
    let node6 = Node {
        id: 6,
        lat: 0,
        lon: 0,
    };
    let node7 = Node {
        id: 7,
        lat: 0,
        lon: 0,
    };
    let node8 = Node {
        id: 8,
        lat: 0,
        lon: 0,
    };
    let node9 = Node {
        id: 9,
        lat: 0,
        lon: 0,
    };
    let node10 = Node {
        id: 10,
        lat: 0,
        lon: 0,
    };
    let node11 = Node {
        id: 11,
        lat: 0,
        lon: 0,
    };
    let node12 = Node {
        id: 12,
        lat: 0,
        lon: 0,
    };
    let node13 = Node {
        id: 13,
        lat: 0,
        lon: 0,
    };
    //based on Professor Bast's example graph from CH lecture 1 slide 16
    let roads = RoadNetwork {
        nodes: HashMap::from([
            (1, node1),
            (2, node2),
            (3, node3),
            (4, node4),
            (5, node5),
            (6, node6),
            (7, node7),
            (8, node8),
            (9, node9),
            (10, node10),
            (11, node11),
            (12, node12),
            (13, node13),
        ]),
        edges: HashMap::from([
            (
                1,
                HashMap::from([(2, (3, false)), (13, (4, false)), (6, (7, false))]),
            ),
            (
                2,
                HashMap::from([(1, (3, false)), (8, (2, false)), (13, (5, false))]),
            ),
            (
                3,
                HashMap::from([(5, (5, false)), (12, (2, false)), (4, (4, false))]),
            ),
            (
                4,
                HashMap::from([(3, (4, false)), (12, (3, false)), (7, (4, false))]),
            ),
            (
                5,
                HashMap::from([(6, (6, false)), (11, (3, false)), (3, (5, false))]),
            ),
            (
                6,
                HashMap::from([(1, (7, false)), (10, (4, false)), (5, (6, false))]),
            ),
            (
                7,
                HashMap::from([(9, (7, false)), (11, (3, false)), (4, (4, false))]),
            ),
            (
                8,
                HashMap::from([(2, (2, false)), (13, (2, false)), (9, (5, false))]),
            ),
            (
                9,
                HashMap::from([(8, (5, false)), (10, (3, false)), (7, (7, false))]),
            ),
            (
                10,
                HashMap::from([
                    (9, (3, false)),
                    (11, (1, false)),
                    (6, (4, false)),
                    (13, (1, false)),
                ]),
            ),
            (
                11,
                HashMap::from([
                    (7, (3, false)),
                    (12, (1, false)),
                    (5, (3, false)),
                    (10, (1, false)),
                ]),
            ),
            (
                12,
                HashMap::from([(4, (3, false)), (3, (2, false)), (11, (1, false))]),
            ),
            (
                13,
                HashMap::from([
                    (8, (2, false)),
                    (10, (1, false)),
                    (1, (4, false)),
                    (2, (5, false)),
                ]),
            ),
        ]),
        raw_ways: vec![Way {
            id: 0,
            speed: 0,
            refs: vec![0, 0],
        }],
        raw_nodes: vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
    };
    let mut graph = RoadDijkstra::new(&roads);
    let mut ch_algo = ContractedGraph::new();
    graph.reset_all_flags(true);

    //(#shortcuts, edge difference = #shortcuts - arcs incident to node)
    print!(
        "edge diff only{:?}\t",
        ch_algo.contract_node(2, &mut graph, true)
    );

    print!(
        "contract node {:?}\t",
        ch_algo.contract_node(2, &mut graph, false)
    );

    let mut graph = RoadDijkstra::new(&roads);
    let mut ch_algo = ContractedGraph::new();
    graph.reset_all_flags(true);

    println!("total shortcuts: {}", ch_algo.ch_precompute(&mut graph));
    println!("{:?}", graph.graph.edges);
    println!("{:?}", ch_algo.ordered_nodes);
    let result = ContractedGraph::bidirectional_compute(&mut graph, 1, 4);
    println!("cost: {}    joint: {}", result.0, result.1);
}
