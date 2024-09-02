fn main() {
    use std::time::Instant;
    use transit_router::{
        road_network::*, transfer_patterns::*, transit_dijkstras::*, transit_network::*,
    };

    use transit_router::RoadNetwork;

    let now = Instant::now();
    let gtfs = read_from_gtfs_zip("hawaii.zip");
    let (transit_graph, connections) = TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10);
    let time = now.elapsed().as_secs_f32();
    println!("time {}", time);
    println!("# of nodes: {}", transit_graph.nodes.len());
    println!(
        "# of edges: {}",
        transit_graph
            .edges
            .values()
            .map(|edges| edges.len())
            .sum::<usize>()
    );

    let now = Instant::now();
    let graph = transit_graph.reduce_to_largest_connected_component();
    let time = now.elapsed().as_secs_f32();

    println!("time {}", time);
    println!("# of nodes: {}", graph.nodes.len());
    println!(
        "# of edges: {}",
        graph.edges.values().map(|edges| edges.len()).sum::<usize>()
    );

    let path = "hawaii.pbf";
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
    let time = now.elapsed().as_millis() as f32 * 0.001;
    println!(
        "time: {}, reduced map nodes: {}, edges: {}",
        time,
        roads.nodes.len(),
        roads.edges.values().map(|edges| edges.len()).sum::<usize>() / 2
    );

    let mut router = Dijkstra::new(&graph);
    let mut transfer_patterns = TransferPatterns::new();

    let now = Instant::now();

    let (source, target) =
        TransferPatterns::make_points_from_coords(21.3732, -157.9201, 21.3727, -157.9172);

    //bus comes at 24480 at Ulune St + Kahuapaani St (Stop ID: 1996) at least in modern day
    let graph = transfer_patterns.query_graph_construction_from_geodesic_points(
        &mut router,
        source,
        target,
        24400,
        1000.0,
    );
    TransferPatterns::query_graph_search(
        roads,
        connections,
        graph.2,
        source,
        target,
        graph.0,
        graph.1,
    );

    println!("time: {:?}", now);
}

#[cfg(test)]
mod tests {
    use transit_router::{transfer_patterns::*, transit_dijkstras::*, transit_network::*};
    //use std::collections::HashMap;
    use std::time::Instant;

    #[test]
    fn test() {
        //Pareto-set ordering
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

        //Direct-Connection Query Test
        /*let connections = DirectConnections {
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
        let query = direct_connection_query(connections, 97, 111, 37200);
        println!("{:?}", query);*/

        let now = Instant::now();
        let gtfs = read_from_gtfs_zip("hawaii.zip");
        let graph = TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10).0;
        let time = now.elapsed().as_secs_f32();

        println!("time {}", time);
        println!("# of nodes: {}", graph.nodes.len());
        println!(
            "# of edges: {}",
            graph.edges.values().map(|edges| edges.len()).sum::<usize>()
        );

        let now = Instant::now();
        let graph = graph.reduce_to_largest_connected_component();
        let time = now.elapsed().as_secs_f32();

        println!("time {}", time);
        println!("# of nodes: {}", graph.nodes.len());
        println!(
            "# of edges: {}",
            graph.edges.values().map(|edges| edges.len()).sum::<usize>()
        );

        let mut precomp_time_per_station = Vec::new();
        let mut histogram_tp: Vec<i32> = vec![0, 0, 0, 0, 0, 0];

        let mut router = Dijkstra::new(&graph);
        let transfer_patterns = TransferPatterns::new();
        //transfer_patterns.hub_selection(&mut router, 1000); //panic call none 547 ln
        let mut total_pairs_considered = 0;

        for i in 0..1 {
            println!("{}", i);
            let source_id = router.get_random_start().unwrap();
            let now = Instant::now();
            let result = transfer_patterns.num_transfer_patterns_from_source(
                source_id.station_id,
                &mut router,
                &None,
            );
            let time = now.elapsed().as_secs_f32();
            precomp_time_per_station.push(time);

            for (_, &ref tp_num) in result.iter() {
                if tp_num.len() == 2 {
                    histogram_tp[0] += 1;
                } else if (3..=6).contains(&tp_num.len()) {
                    histogram_tp[1] += 1;
                } else if (7..=11).contains(&tp_num.len()) {
                    histogram_tp[2] += 1;
                } else if (12..=21).contains(&tp_num.len()) {
                    histogram_tp[3] += 1;
                } else if (22..=51).contains(&tp_num.len()) {
                    histogram_tp[4] += 1;
                } else if tp_num.len() >= 52 {
                    histogram_tp[5] += 1;
                }
            }
            total_pairs_considered += result.len();
        }

        println!(
            "average station preprocess time in seconds {}",
            precomp_time_per_station.iter().sum::<f32>() / precomp_time_per_station.len() as f32
        );

        for i in histogram_tp.iter_mut() {
            *i = *i * 100 / total_pairs_considered as i32;
        }

        println!(
            "number of transfer patterns histogram percent {:?}",
            histogram_tp
        );

        /*let now = Instant::now();
        let gtfs = read_from_gtfs_zip("manhattan.zip");
        let graph = TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10).0;
        let time = now.elapsed().as_secs_f32();

        println!("time {}", time);
        println!("# of nodes: {}", graph.nodes.len());
        println!(
            "# of edges: {}",
            graph
                .edges
                .iter()
                .map(|(_, edges)| edges.len())
                .sum::<usize>()
        );

        //1,831 / 830,100 / 1,371,298     2s     5ms     2h12m35s
        let now = Instant::now();
        let graph = graph.reduce_to_largest_connected_component();
        let mut time = now.elapsed().as_secs_f32();

        println!("time {}", time);
        println!("# of nodes: {}", graph.nodes.len());
        println!(
            "# of edges: {}",
            graph
                .edges
                .iter()
                .map(|(_, edges)| edges.len())
                .sum::<usize>()
        );

        let mut query_time = Vec::new();
        let mut shortest_path_costs = Vec::new();

        let mut routing_graph = Dijkstra::new(&graph);
        for _ in 0..1000 {
            let source_id = routing_graph.get_random_start().unwrap();
            let target_id = Some(
                routing_graph
                    .get_random_end(source_id.time.unwrap())
                    .unwrap(),
            );
            let now = Instant::now();
            let result = routing_graph.time_expanded_dijkstra(Some(source_id), None, target_id, &None, &None);
            time = now.elapsed().as_millis() as f32;
            query_time.push(time);

            if let Some(result) = result {
                let cost = result.cost_from_start;
                shortest_path_costs.push(cost);
            }

            //println!("{} cost {}", x, cost);
        }

        println!(
            "average query time in miliseconds {}",
            query_time.iter().sum::<f32>() / query_time.len() as f32
        );

        println!(
            "average cost hh::mm:ss {}:{}:{} \n",
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 / 3600,
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 % 3600 / 60,
            shortest_path_costs.iter().sum::<u64>() / shortest_path_costs.len() as u64 % 60,
        );*/

        /*let mut edges: HashMap<i64, HashMap<i64, (u64, bool)>> = HashMap::new();
        let mut station = vec![
            (755, 1),
            (755, 1),
            (800, 2),
            (800, 2),
            (800, 3),
            (800, 3),
            (802, 2),
            (807, 2),
            (807, 3),
            (812, 3),
            (817, 2)
        ];
        station.sort_by(|a, b| a.0.cmp(&b.0));
        let time_chunks = station.chunk_by_mut(|a, b| a.0 == b.0);

        let mut station_nodes_by_time: Vec<(u64, i64)> = Vec::new();

        for chunk in time_chunks {
            chunk.sort_by(|a, b| (a.1).cmp(&(b.1)));
            station_nodes_by_time.append(&mut chunk.to_vec().to_owned())
        }
        println!("{:?}", station_nodes_by_time);
        let mut prev_transfer_node: Option<(u64, i64)> = None;
        let mut current_index: usize = 0;
        for node in station_nodes_by_time.iter() {
            println!("{:?}", node);
            current_index += 1;
            if node.1 == 2 {
                if let Some((prev_tranfer_time, prev_transfer_id)) = prev_transfer_node {
                    println!("waiting");
                    edges //waiting (arrival to transfer)
                        .entry(prev_transfer_id) //tail
                        .and_modify(|inner| {
                            inner.insert(node.1, (node.0 - prev_tranfer_time, true));
                            //head
                        })
                        .or_insert({
                            let mut a = HashMap::new();
                            a.insert(node.1, (node.0 - prev_tranfer_time, true)); //head
                            a
                        });
                }

                prev_transfer_node = Some(*node);
                println!("new tf {:?}", prev_transfer_node);

                for index in current_index..station_nodes_by_time.len() {
                    let node = station_nodes_by_time.get(index).unwrap();
                    println!("l{:?}", node);

                    if node.1 == 2 {
                        break;
                    }

                    if node.1 == 3 {
                        if let Some((prev_tranfer_time, prev_transfer_id)) = prev_transfer_node {
                            println!("boarding");
                            edges //boarding (arrival to transfer)
                                .entry(prev_transfer_id) //tail
                                .and_modify(|inner| {
                                    inner.insert(node.1, (node.0 - prev_tranfer_time, true));
                                    //head
                                })
                                .or_insert({
                                    let mut a = HashMap::new();
                                    a.insert(node.1, (node.0 - prev_tranfer_time, true)); //head
                                    a
                                });
                            prev_transfer_node = Some(*node)
                        }
                    }
                }
            }
        }
        println!("{:?}", edges); */

        let now = Instant::now();
        let gtfs = read_from_gtfs_zip("test.zip");
        let graph = TimeExpandedGraph::new(gtfs, "Wednesday".to_string(), 10).0;
        let mut router = Dijkstra::new(&graph);
        let transfer_patterns = TransferPatterns::new();

        println!("{:?}\n# of nodes: {}", now, graph.nodes.len());
        println!(
            "# of edges: {}",
            graph
                .edges
                .iter()
                .map(|(_, edges)| edges.len())
                .sum::<usize>()
        );

        //println!("edges {:?}\n", router.graph.edges);

        println!("stations {:?}\n", router.graph.station_mapping);

        let &source_id = router.graph.station_mapping.get("A").unwrap();
        //let &target_id = router.graph.station_mapping.get("F").unwrap();

        let result = transfer_patterns.num_transfer_patterns_from_source(source_id, &mut router, &None);

        println!("aa{:?}", result);
        //let now = Instant::now();

        //transfer_patterns.hub_selection(&mut router, 500, u64::MAX);

        //println!("hubs \n{:?}", transfer_patterns.hubs);
    }
}
