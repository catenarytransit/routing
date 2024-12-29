#![allow(unused)]
// Copyright Chelsea Wen
// Cleaned up somewhat by Kyler Chin
use serde_json::{Result, Value};
use std::fs::File;
use std::io::BufReader;

fn main() {
    /*
    use geo::point;
    use std::collections::HashMap;
    use std::time::Instant;
    use transit_router::coord_int_convert::*;
    use transit_router::road_dijkstras::*;
    use transit_router::transit_dijkstras::TransitDijkstra;
    use transit_router::RoadNetwork;
    use transit_router::{transfer_patterns::*, transit_network::*};

    let now = Instant::now();
    let gtfs = read_from_gtfs_zip("gtfs_stm.zip");
    let (transit_graph, connections) = TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10);

    let mut router = TransitDijkstra::new(&transit_graph);

    println!("time for transit {:?}", now.elapsed());
    let now = Instant::now();

    //read bytes from file ped-and-bike-north-america-canada-quebec.osm.bincode
    //let path = "./ped-and-bike-north-america-canada-quebec.osm.bincode";

    /*read data from file
    let bytes = fs::read(path).unwrap();
    let data = RoadNetwork::from_bincode_file(&bytes);*/

    let data = RoadNetwork::read_from_osm_file("quebec1.pbf").unwrap();
    let mut roads = RoadNetwork::new(data.0, data.1);
    roads = roads.reduce_to_largest_connected_component();

    println!("time for road {:?}", now.elapsed());

    println!("# of nodes: {}", roads.nodes.len());
    println!(
        "# of edges: {}",
        roads.edges.values().map(|edges| edges.len()).sum::<usize>()
    );

    //Gare de Centrale, Montreal, Quebec, Canada
    let source = point! {x:-73.567398, y:45.499860
    };
    //Parc Olympique, Montreal, Quebec, Canada
    let target = point! {
        x:-73.547620, y:45.559989
    };

    let start_time = 32400;

    use rstar::RTree;

    let mut source_paths: HashMap<&NodeId, RoadPathedNode> = HashMap::new();
    let road_node_tree = RTree::bulk_load(
        roads
            .nodes
            .values()
            .map(|n| int_to_coord(n.lon, n.lat))
            .collect(),
    );

    println!("Made rtree");

    let mut graph = RoadDijkstra::new(&roads);

    println!("Starting graph construction");

    let preset_distance = 500.0;

    let now = Instant::now();
    let graph = query_graph_construction_from_geodesic_points(
        &mut router,
        source,
        target,
        //09:00 departure
        32400,
        preset_distance,
    );

    println!("query graph constructed in {:?}", now.elapsed());

    //println!("source nodes {:?}", graph.0);
    //println!("target nodes {:?}", graph.1);

    let run_query = query_graph_search(
        &roads,
        connections,
        graph,
        preset_distance,
    );

    let reverse_station_map = transit_graph
        .station_map
        .iter()
        .map(|(name, id)| (id, name))
        .collect::<HashMap<_, _>>();

    if let Some(stuff) = run_query {
        let path = stuff.2.get_path();
        for node in path.0 {
            println!(
                "path: {}",
                reverse_station_map.get(&node.station_id).unwrap()
            );
        }
    }*/
}