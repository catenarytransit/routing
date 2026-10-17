// Written by C. Wen
#![allow(unused)]

use geo::{Point, point};
use regex::Regex;
use routing::{road_dijkstras::*, road_network::*, raptor::*};
use std::fs::*;
use std::io::*;
use std::time::Instant;

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

    let args = Args::parse();

    if !args.debugmode {
        let preset_distance = 250.0;
        println!("Arguments: {args:#?}");
        let now = Instant::now();
        println!("query graph constructed in {:?}", now.elapsed());
        //let contents: String = ron::to_string().unwrap();

        let savepath = "results.ron";

        println!("generating transit network graph");
        //let timetable = read_timetable_file("timetable.bin");
            
    } 
    //else {
       
    //}
}


#[allow(dead_code)]
fn find_target_section(
    node: road_graph_construction::Node,
    re_child: &Regex,
) -> Option<RoadDijkstra> {
    let dirpath = "arc_regions";
    let entries = read_dir(dirpath)
        .unwrap()
        .map(|res| res.map(|e| e.path()))
        .filter(|x| x.is_ok());
    for entry in entries {
        let file = entry.unwrap();
        let path = file.as_path().to_str().unwrap();
        let mut input = File::open(path).ok().unwrap();
        let mut contents: String = "".to_string();
        if let Some(res) = re_child.captures(path) {
            let min_lat = res.get(1).unwrap().as_str().parse::<f32>().unwrap();
            let min_lon = res.get(2).unwrap().as_str().parse::<f32>().unwrap();
            let max_lat = res.get(3).unwrap().as_str().parse::<f32>().unwrap();
            let max_lon = res.get(4).unwrap().as_str().parse::<f32>().unwrap();
            let coords =
                road_graph_construction::CoordRange::from_deci(min_lat, max_lat, min_lon, max_lon);
            let (lat_range, lon_range) = coords.give_ranges();
            //println!("range {:?} {:?} and node {} {}", lat_range, lon_range, node.lat, node.lon);
            if lat_range.contains(&node.lat) && lon_range.contains(&node.lon) {
                let _ = input.read_to_string(&mut contents);
                let graph: RoadDijkstra = ron::from_str(&contents).unwrap();
                return Some(graph);
            }
        }
    }
    None
}
