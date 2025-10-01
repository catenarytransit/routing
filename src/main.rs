#![allow(unused)]
// Copyright Chelsea Wen
// Cleaned up somewhat by Kyler Chin
use geo::{Point, point};
use regex::Regex;
use routing::{
    road_dijkstras::*,
    road_network::{
        //contraction_hierarchies::*, landmark_algo::*, 
        road_graph_construction::*,
    },
    transfers::*,
    transit_dijkstras::*,
    transit_network::*,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::*;
use std::io::*;
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

/* use clap::Parser;
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Number of times to greet
    #[arg(long, default_value_t = false)]
    makequerygraph: bool,
    #[arg(long, default_value_t = true)]
    debugmode: bool,
}
 */
/*
#[tokio::main]
async fn main() {
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
*/

#[tokio::main]
async fn main() {
    let dirpath = "osm_folder";
    let re = Regex::new(r"(.+)\.").unwrap();
    let entries = read_dir(dirpath)
        .unwrap()
        .map(|res| res.map(|e| e.path()))
        .collect::<Vec<_>>();

    for entry in entries {
        let file = entry.unwrap();
        let path = file.as_path().to_str().unwrap();

        let now = Instant::now();
        let mut time = now.elapsed().as_millis() as f32 * 0.001;
        let data = RoadNetwork::read_from_osm_file(path);
        let mut roads = RoadNetwork::new(data.0, data.1);
        print!(
            "{} Base Graph Nodes: {}, Edges: {}\t\t",
            path,
            roads.nodes.len(),
            roads.edges.len()
        );
        let now = Instant::now();
        roads = roads.reduce_to_largest_connected_component();
        time = now.elapsed().as_millis() as f32 * 0.001;
        println!(
            "time: {}, reduced map nodes: {}, edges: {}",
            time,
            roads.nodes.len(),
            roads.edges.values().map(|edges| edges.len()).sum::<usize>() / 2
        );

        let mut graph = RoadDijkstra::new(&roads);

        println!("query graph constructed in {:?}", now.elapsed());
        let now = Instant::now();
        //let precompute = landmark_heuristic_precompute(&mut graph, 42);

        /*
            take region and divide it into rectangular subregions for arcflags
            for each subregion, take the four corners and plug it into arc_flags_precompute
            neven borders? overestimate, any nodes in the blank region will just simply not exist
            while nodes that stick out will get filtered out
            thus, double check that total rectangle of region is bigger than entire nodefield
            global min and max should be that of all nodes, then divide into subregions from there
        */
        //for chunk of bounds (geographic rectangle or something) generate arcflags maps {
            //do all the code below
        let bounds = CoordRange::new(49.20, 49.25, 6.95, 7.05);
        let boundstr= arc_flags_precompute(bounds, &mut graph); //saar
        println!("arc flags set in {:?}", now.elapsed());
        
        let filename = re.captures(path).unwrap().extract::<1>().0;
        let savepath = format!("{filename}{boundstr}json");
        let mut output = File::create(savepath.clone()).unwrap();
        
        let contents: String = ron::to_string(&graph).unwrap();
        write!(output, "{contents}");

        /*
            let mut ch_algo = ContractedGraph::new();
            let now = Instant::now();

            graph.reset_all_flags(true);
            graph.set_max_settled_nodes(20);

            ch_algo.compute_random_node_ordering(&mut graph, 1000); //here
            let mut contraction_time = Vec::new();
            let mut shortcut_hg = vec![0, 0, 0, 0, 0];
            let mut edge_diff_hg = vec![0, 0, 0, 0, 0];
            for (&n, _) in ch_algo.ordered_nodes.clone().iter() {
                let now = Instant::now();
                let (num_shortcut, num_edge_diff) = ch_algo.contract_node(n, &mut graph, false);
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

            println!("shortcut histogram {:?}", shortcut_hg);

            println!("edge difference histogram {:?}", edge_diff_hg);


            graph.reset_all_flags(true);

            let total_shortcuts = ch_algo.ch_precompute(&mut graph);
            time = now.elapsed().as_millis() as f32 * 0.001;
            println!("precomp seconds: {}", time);
            println!("total shortcuts: {}", total_shortcuts);

            let mut shortest_path_costs = Vec::new();
            let mut query_time = Vec::new();

            for _ in 0..1000 {
                //1000
                let source = graph.get_random_node_id().unwrap();
                let target = graph.get_random_node_id().unwrap();
                let now = Instant::now();
                let result = ContractedGraph::bidirectional_compute(&mut graph, source, target);
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
        */

        /*
        TODO: Given OSM section X,  Y number of arc zones, and coordinate pair Z to chunk into:
            🗸 Write algo to chunk given OSM section into Y arc zones
            🗸 Precompute RoadDijkstra graph for section OSM section X --> Save with serde as json --> Compress!
            🗸 Precompute arcflags for each arc zone Y --> Save with serde as json --> Compress!
            - For coordinate pair Z, locate the OSM section X' for Z_s and Z_t and arc zone Y' for Z_t
            🗸 Calculate a_star_heurstic for Z_t
            🗸 Query with dijkstras
        Question: How to connect between different OSM sections? Arc Flag hierarchy similar to CH algo? Go from top down
            --> between OSM sections and then between individual sectors?
        */

        //let arc_flag_thing = ArcFlags::new(47.95, 48.05, 7.75, 7.90); //ba-wu
        //let arc_flag_thing = ArcFlags::new(33.63, 33.64, -117.84, -117.83); //uci

        let mut input = File::open(savepath.as_str()).ok().unwrap();
        let mut contents: String = "".to_string();
        let reader = input.read_to_string(&mut contents);
        let mut graph: RoadDijkstra = ron::from_str(&contents).unwrap();

        let mut shortest_path_costs = Vec::new();
        let mut query_time = Vec::new();
        let mut settled_nodes = Vec::new();
        let mut heuristics = None;

        /*
        hashmap of id of a subregion's RON and the lon/lat ranges it represents?
        given a target node, search the hashmap and retrieve the id of the subregion it's inside of
        read graph from that RON and route from the source to that target
        
        what about inter-region routing?
        uh, something with the border nodes potentially? route from source to closest border node of target region
        then follow into the subregion routing?
        or maybe have a tier of different arcflags on a top-down level like contraction hiarchies --> is it worth it?
        */
        for _ in 0..100 {
            let source = graph.get_random_node_id().unwrap();
            //let target = graph.get_random_node_id().unwrap();
            let target = graph.get_random_node_area_id(49.20, 49.25, 6.95, 7.05); //saar
            //let target = graph.get_random_node_area_id(47.95, 48.05, 7.75, 7.90); //ba-wu
            //let target = graph.get_random_node_area_id(33.63, 33.64, -117.84, -117.83); //uci

            //heuristics = landmark_heuristic(&precompute, &graph, target);

            let now = Instant::now();
            heuristics = Some(a_star_heuristic(&graph.graph, target));
            let result = graph.dijkstra(source, target, &heuristics, true);
            time = now.elapsed().as_millis() as f32 * 0.001;
            query_time.push(time);

            if let Some(cost) = result.0 {
                let result = cost.get_path();
                shortest_path_costs.push(result.1);
            } else {
                print!("f");
                shortest_path_costs.push(0);
            }
            settled_nodes.push(graph.visited_nodes.len() as u64);
        }

        println!(
            "average travel time in minutes {}",
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 / 60000
        );
        println!(
            "average query time in seconds {}",
            query_time.iter().sum::<f32>() / query_time.len() as f32
        );
        println!(
            "average settle node number {}",
            settled_nodes.iter().sum::<u64>() / settled_nodes.len() as u64
        );
    }
}
