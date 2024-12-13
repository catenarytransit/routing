#![allow(unused)]
// Copyright Chelsea Wen
// Cleaned up somewhat by Kyler Chin
use std::fs::File;
use serde_json::{Result, Value};
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

    let reverse_station_mapping = transit_graph
        .station_mapping
        .iter()
        .map(|(name, id)| (id, name))
        .collect::<HashMap<_, _>>();

    if let Some(stuff) = run_query {
        let path = stuff.2.get_path();
        for node in path.0 {
            println!(
                "path: {}",
                reverse_station_mapping.get(&node.station_id).unwrap()
            );
        }
    }*/
}

#[cfg(test)]
mod tests {
    use geo::point;
    use std::collections::HashMap;
    use std::env;
    use std::f64::consts;
    use std::time::Instant;
    use transit_router::coord_int_convert::coord_to_int;
    use transit_router::NodeType;
    use transit_router::RoadNetwork;
    use transit_router::{transfer_patterns::*, transit_dijkstras::*, transit_network::*};
    use std::fs::*;
    use std::io::Write;
    use std::io::BufReader;


    #[test]
    fn test() {
        let now = Instant::now();
        let savepath = "results.json";

        println!("generating transit network graph");
        let gtfs = read_from_gtfs_zip("ctt.zip");
        let (transit_graph, connections) =
            TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10);
        let mut router = TransitDijkstra::new(&transit_graph);

        //connection tests, these should be valid transfers --> success!
        /*
        //https://maps.app.goo.gl/eXf4S5edPM8vgvVt9
        println!("time for transit {:?}", now.elapsed());
        let start_station = *transit_graph.station_mapping.get("9079").unwrap(); //Blue Hills Ave @ Home Goods
        let end_station = *transit_graph.station_mapping.get("1682").unwrap(); //Bloomfield Ave @ Advo

        let x = direct_connection_query(&connections, start_station, end_station, 25680);//7:08 AM
        println!("dc query {:?}", x);

        //https://maps.app.goo.gl/szffQJAEALqSeHNF7
        let source_id = NodeId { //Downtown New Britain Station @ Columbus Blvd Bay
            node_type: NodeType::Arrival,
            station_id: 13381,
            time: Some(19200), //5:20 AM, 10 second transfer buffer
            trip_id: 1758411,
        };
        let target_id = NodeId { //Hart St @ Camp St
            node_type: NodeType::Arrival,
            station_id: 9738,
            time: Some(19515),
            trip_id: 1758411,
        };

        let d = router.time_expanded_dijkstra(Some(source_id), None, Some(target_id), None);


        println!("routing {:#?} and visted count {}", d.0.unwrap().get_path(), d.1.len());
        */

        //full routing test
        //see following link, anything but first option (which includes walking between stations, hasnt been implemented yet)
        //https://www.google.com/maps/dir/Bloomfield,+Connecticut+06002/77+Forest+St,+Hartford,+CT+06105/@41.823207,-72.7745391,34082m/data=!3m1!1e3!4m20!4m19!1m5!1m1!1s0x89e7001af40714d7:0xc4be608b22d7e4a8!2m2!1d-72.7197095!2d41.8683576!1m5!1m1!1s0x89e653502e880197:0xc1f0096f7d179457!2m2!1d-72.7005256!2d41.7671825!2m4!4e3!6e0!7e2!8j1727241000!3e3!5i1

        let preset_distance = 250.0;
        
        //pepperidge farm to harriet beecher stowe center
        let (source, target) = make_points_from_coords(
            -72.71973332600558,
            41.86829675142084,
            -72.70049435551549,
            41.76726348091365,
        );

        let now = Instant::now();
        let graph = query_graph_construction_from_geodesic_points(
            &mut router,
            source,
            target,
            18000,
            18600, //5:10 AM
            preset_distance,
        );


        let mut output = File::create(savepath).unwrap();
        println!("query graph constructed in {:?}", now.elapsed());
        serde_json::to_writer(output, &graph).unwrap();

        //part 2
        /*let file = File::open(savepath).ok().unwrap();
        let reader = BufReader::new(file);
        let mut graph: QueryGraphItem = serde_json::from_reader(reader).unwrap();

        //road network, for footpaths
        let now = Instant::now();
        println!("generating street network graph");
        let path = "ct.pbf";
        let data = RoadNetwork::read_from_osm_file(path).unwrap();
        let mut roads = RoadNetwork::new(data.0, data.1);
        //roads = roads.reduce_to_largest_connected_component();

        println!("time for road {:?}", now.elapsed());

        println!("# of nodes: {}", roads.nodes.len());
        println!(
            "# of edges: {}",
            roads.edges.values().map(|edges| edges.len()).sum::<usize>()
        );

        let run_query = query_graph_search(&roads, connections, graph);

        let reverse_station_mapping = transit_graph
            .station_mapping
            .iter()
            .map(|(name, id)| (id, name))
            .collect::<HashMap<_, _>>();

        println!("path: \t");
        //i think the hub pathway is broken, need to verify that the path from the source to hub is being extended correctly
        if let Some(stuff) = run_query {
            let path = stuff.2.get_path();
            for node in path.0 {
                print!(
                    "{},",
                    reverse_station_mapping.get(&node.station_id).unwrap()
                );
            }
        }
        println!(".");*/

        //Pareto-se t ordering
        /*fn pareto_recompute(set: &mut Vec<(i32, i32)>, c_p: (i32, i32)) {
            set.sort_by_key(|(x, y)| if y != &0 { 10 * x + (10 / y) } else { 10 * x });
            let mut incomparable = true;
            for c in set.iter() {
                if (c.0 <= c_p.0 && c.1 <= c_p.1) || (c.0 >= c_p.0 && c.1 >= c_p.1) {
                    incomparable = false;
                }

            }
            if incomparable {
                set.push(c_p);
                set.dedup();
                set.sort_by_key(|(x, y)| if y != &0 { 10 * x + (10 / y) } else { 10 * x });
            }
        }

        let mut set = vec![(1, 5), (7, 1), (2, 4), (4, 3)];
        pareto_recompute(&mut set, (6, 2));
        println!("{:?}", set);
        pareto_recompute(&mut set, (1, 1));
        println!("{:?}", set);
        pareto_recompute(&mut set, (8, 0));
        println!("{:?}", set);
        pareto_recompute(&mut set, (8, 0));
        println!("{:?}\n", set);

        let mut set = vec![(9, 1), (8, 1), (2, 7), (3, 3), (6, 4), (1, 9)];
        pareto_recompute(&mut set, (5, 5));
        println!("{:?}", set);
        pareto_recompute(&mut set, (7, 3));
        println!("{:?}", set);
        pareto_recompute(&mut set, (7, 2));
        println!("{:?}", set);
        */

        //early stage Direct-Connection Query Test
        /*let connections = DirectConnections {
            route_tables: HashMap::from([("L17".to_string(), LineConnectionTable {
                times_from_start: HashMap::from([(154,0), (97,420), (987, 720), (111,1260)]),
                start_times: Vec::from([29700, 33300, 36900, 40800, 44400])
            })]),
            lines_per_station: HashMap::from([(97, Vec::from([
                    ("L8".to_string(), 4), ("L17".to_string(), 2),("L34".to_string(), 5), ("L87".to_string(), 17)])),
                (111, Vec::from([
                    ("L9".to_string(), 1), ("L13".to_string(), 5),("L17".to_string(), 4), ("L55".to_string(), 16)
                ]))])
        };
        let query = direct_connection_query(&   connections, 97, 111, 37200);
        println!("aa query results: {:?}", query);*/
    }
}
