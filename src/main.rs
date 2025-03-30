#![allow(unused)]
// Copyright Chelsea Wen
// Cleaned up somewhat by Kyler Chin
use geo::{point, Point};
use serde_json::{Result, Value};
use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::time::Instant;
use tokio::*;
use transit_router::{transfers::*, transit_dijkstras::*, transit_network::*};

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
    #[arg(long, default_value_t = true)]
    makequerygraph: bool,
    #[arg(long, default_value_t = true)]
    debugmode: bool,
}

#[tokio::main]
async fn main() {
    let mut args = Args::parse();
    let preset_distance = 250.0;

    if !args.debugmode {
        args.makequerygraph = false;

        println!("Arguments: {:#?}", args);

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

        if let Some(stuff) = run_query {
            let path = stuff.1.get_tp(stuff.0, &paths, &maps);
            for (node, route) in path.0 {
                println!("{:?}", node);
                if let Some(route) = route {
                    println!("via {:?}", routes.get(&route).unwrap().short_name);
                } else {
                    println!("start {}", stops.get(&node.station_id.to_string()).unwrap());
                }
            }
        } else {
            println!("no path");
        }
    } else {
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

        let run_query = query_graph_search(connections, graph, &mut paths);

        if let Some(stuff) = run_query {
            let path = stuff.1.get_tp(stuff.0, &paths, &maps);
            for (node, route) in path.0 {
                println!("{:?}", node);
                if let Some(route) = route {
                    println!("via {:?}", route);
                } else {
                    println!("start {:?}", maps.station_num_to_name(&node.station_id));
                }
            }
        } else {
            println!("no path");
        }
    }
}
