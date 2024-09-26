#![allow(unused)]
// Copyright Chelsea Wen
// Cleaned up somewhat by Kyler Chin
use std::fs;

fn main() {
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
    }
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

    #[test]
    fn test() {
        let now = Instant::now();
        let gtfs = read_from_gtfs_zip("ctt.zip");
        let (transit_graph, connections) =
            TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10);
        let mut router = TransitDijkstra::new(&transit_graph);

        println!("time for transit {:?}", now.elapsed());
        let start_station = *transit_graph.station_mapping.get("9079").unwrap();
        let end_station = *transit_graph.station_mapping.get("1682").unwrap();

        let x = direct_connection_query(&connections, start_station, end_station, 25680);
        println!("result {:?}", x);

        let source_id = NodeId {
            node_type: NodeType::Transfer,
            station_id: 13381,
            time: Some(19210),
            trip_id: 1758411,
        };
        let target_id = NodeId {
            node_type: NodeType::Arrival,
            station_id: 9738,
            time: Some(19515),
            trip_id: 1758411,
        };

        let d = router.time_expanded_dijkstra(Some(source_id), None, Some(target_id), None);

        println!("routing {:?}", d);

        let now = Instant::now();
        let path = "ct.pbf";
        let data = RoadNetwork::read_from_osm_file(path).unwrap();
        let mut roads = RoadNetwork::new(data.0, data.1);
        roads = roads.reduce_to_largest_connected_component();

        println!("time for road {:?}", now.elapsed());

        println!("# of nodes: {}", roads.nodes.len());
        println!(
            "# of edges: {}",
            roads.edges.values().map(|edges| edges.len()).sum::<usize>()
        );
        let (source, target) = make_points_from_coords(
            -72.71973332600558,
            41.86829675142084,
            -72.70049435551549,
            41.76726348091365,
        );

        let preset_distance = 250.0;

        let now = Instant::now();
        let graph = query_graph_construction_from_geodesic_points(
            &mut router,
            source,
            target,
            18600,
            preset_distance,
        );

        println!("query graph constructed in {:?}", now.elapsed());

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

        print!("path: \t");
        if let Some(stuff) = run_query {
            let path = stuff.2.get_path();
            for node in path.0 {
                print!(
                    "{},",
                    reverse_station_mapping.get(&node.station_id).unwrap()
                );
            }
        }
        println!(".");

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

        /*let now = Instant::now();
        let gtfs = read_from_gtfs_zip("ctt.zip");
        let graph = TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10).0;
        let time = now.elapsed().as_secs_f32();

        println!("time {}", time);
        println!("# of nodes: {}", graph.nodes.len());
        println!(
            "# of edges: {}",
            graph.edges.values().map(|edges| edges.len()).sum::<usize>()
        );

        let mut router = TransitDijkstra::new(&graph);
        let hubs = hub_selection(&router, 10000, 54000);
        router.node_deactivator(&hubs);
        let mut precomp_time_per_station = Vec::new();

        for h in 0..10 {
            let h = router.get_random_start().unwrap().station_id;
            let now = Instant::now();
            let result = num_transfer_patterns_from_source(h, &router, Some(&hubs));
            let time = now.elapsed().as_secs_f32();
            precomp_time_per_station.push(time);
        }

        println!(
            "average station preprocess time in seconds {}",
            precomp_time_per_station.iter().sum::<f32>() / precomp_time_per_station.len() as f32
        );*/

        /*let now = Instant::now();
        let gtfs = read_from_gtfs_zip("test.zip");
        let graph = TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10).0;
        let mut router = TransitDijkstra::new(&graph);

        println!("{:?}\n# of nodes: {}", now, graph.nodes.len());
        println!(
            "# of edges: {}",
            graph.edges.values().map(|edges| edges.len())
                .sum::<usize>()
        );

        //println!("edges {:?}\n", router.graph.edges);

        println!("stations {:?}\n", router.graph.station_mapping);

        let &source_id = router.graph.station_mapping.get("A").unwrap();
        //let &target_id = router.graph.station_mapping.get("F").unwrap();

        let hubs = hub_selection(&router, 1, 60);
        let result = num_transfer_patterns_from_source(source_id, &mut router, Some(&hubs));

        println!("aa{:?}", result);

        //println!("hubs \n{:?}", transfer_patterns.hubs);*/

        /*
        //Direct-Connection Query Test
        let connections = DirectConnections {
            route_tables: HashMap::from([("L17".to_string(), LineConnectionTable {
                route_id: "L17".to_string(),
                times_from_start: HashMap::from([(154, (0, 1)), (97, (420, 2)), (987, (720, 3)), (111, (1260, 4))]),
                start_times: Vec::from([297000,33300, 36900, 40800, 44400])
            })]),
            lines_per_station: HashMap::from([(97, HashMap::from([
                    ("L8".to_string(), 4), ("L17".to_string(), 2),("L34".to_string(), 5), ("L87".to_string(), 17)])),
                (111, HashMap::from([
                    ("L9".to_string(), 1), ("L13".to_string(), 5),("L17".to_string(), 4), ("L55".to_string(), 16)
                ]))])
        };
        let query = direct_connection_query(&   connections, 97, 111, 37200);
        println!("query results: {:?}", query);
         */
    }
}
