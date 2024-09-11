#[allow(unused)]
pub mod road_graph_construction {
    //constructs and preprocesses the graph struct from OSM data
    use crate::coord_int_convert::TEN_TO_14;
    use crate::road_dijkstras::*;
    use core::num;
    use geo::HaversineDistance;
    use osmpbfreader::objects::OsmObj;
    use serde::{Deserialize, Serialize};
    use std::{collections::HashMap, ops::Index};

    #[derive(Debug, PartialEq, Hash, Eq, Clone, Copy, PartialOrd, Ord)]
    pub struct Node {
        //nodes from OSM, each with unique ID and coordinate position
        pub id: i64,
        pub lat: i64,
        pub lon: i64,
    }

    pub struct I64Point {
        //used to store coordinates of nodes
        pub lat: i64,
        pub lon: i64,
    }

    #[derive(Debug, PartialEq, Hash, Eq, Clone)]
    pub struct Way {
        //ways from OSM, each with unique ID, speed from highway type, and referenced nodes that it connects
        pub id: i64,
        pub speed: u16,
        pub refs: Vec<i64>,
    }

    #[derive(Debug, PartialEq, Clone)]
    pub struct RoadNetwork {
        //graph struct that will be used to route
        pub nodes: HashMap<i64, Node>,              // <node.id, node>
        pub edges: HashMap<i64, HashMap<i64, u64>>, // tail.id, <head.id, cost>
        pub raw_ways: Vec<Way>,
        pub nodes_by_coords: HashMap<(i64, i64), i64>,
    }

    pub fn speed_calc(highway: &str) -> Option<u16> {
        //for pedestrians
        //picks speed of highway based on given values, in km/h
        match highway {
            "pedestrian" => Some(4),
            "path" => Some(4),
            "footway" => Some(4),
            "steps" => Some(4),
            "corridor" => Some(4),
            "living_street" => Some(4),
            "sidewalk" => Some(4),
            "traffic_island" => Some(4),
            "crossing" => Some(3),
            "road" => Some(4),
            "unclassified" => Some(4),
            "residential" => Some(4),
            "unsurfaced" => Some(4),
            "living_street" => Some(4),
            "service" => Some(4),
            "trunk" => Some(4),
            "primary" => Some(4),
            "secondary" => Some(4),
            "tertiary" => Some(4),
            "motorway_link" => Some(4),
            "trunk_link" => Some(4),
            "primary_link" => Some(4),
            "secondary_link" => Some(4),
            _ => Some(1),
        }
    }

    #[derive(serde::Serialize, serde::Deserialize, Clone)]
    struct ExportOsm {
        nodes: Vec<osmpbfreader::Node>,
        ways: Vec<osmpbfreader::Way>,
    }

    impl RoadNetwork {
        pub fn new(mut nodes: HashMap<i64, Node>, ways: Vec<Way>) -> Self {
            //init new RoadNetwork based on results from reading .pbf file
            let mut edges: HashMap<i64, HashMap<i64, u64>> = HashMap::new();
            for way in ways.iter() {
                let mut previous_head_node_now_tail: Option<&Node> = None;
                let mut previous_head_node_index: usize = 0;
                for i in 0..way.refs.len() - 1 {
                    let tail_id = way.refs[i];
                    let tail: Option<&Node> = match previous_head_node_now_tail {
                        Some(previous_head_node_now_tail) => match previous_head_node_index == i {
                            true => Some(previous_head_node_now_tail),
                            false => nodes.get(&tail_id),
                        },
                        None => nodes.get(&tail_id),
                    };

                    let head_id = way.refs[i + 1];
                    let head = nodes.get(&head_id);
                    if let (Some(tail), Some(head)) = (tail, head) {
                        let head_point = geo::Point::new(
                            (head.lon as f64 / TEN_TO_14),
                            (head.lat as f64 / TEN_TO_14),
                        );
                        let tail_point = geo::Point::new(
                            (tail.lon as f64 / TEN_TO_14),
                            (tail.lat as f64 / TEN_TO_14),
                        );

                        let distance = head_point.haversine_distance(&tail_point);

                        let cost = (distance / ((way.speed as f64) * 5.0 / 18.0)) as u64; //seconds to traverse segment based on road type
                        let flag = false;
                        edges
                            .entry(tail_id)
                            .and_modify(|inner| {
                                inner.insert(head_id, cost);
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(head_id, cost);
                                a
                            });
                        edges
                            .entry(head.id)
                            .and_modify(|inner| {
                                inner.insert(tail_id, cost);
                            })
                            .or_insert({
                                let mut a = HashMap::new();
                                a.insert(tail_id, cost);
                                a
                            });
                        previous_head_node_now_tail = Some(head);
                        previous_head_node_index = i + 1;
                    }
                }
            }
            let node_to_remove = nodes
                .iter()
                .filter(|(node, _)| !edges.contains_key(node))
                .map(|(x, _)| *x)
                .collect::<Vec<i64>>();
            for node in &node_to_remove {
                nodes.remove(node);
            }

            let nodes_by_coords = nodes
                .iter()
                .map(|(id, node_info)| ((node_info.lon, node_info.lat), *id))
                .collect();

            Self {
                raw_ways: ways,
                edges,
                nodes_by_coords,
                nodes,
            }
        }

        pub fn from_bincode_file(data: &[u8]) -> (HashMap<i64, Node>, Vec<Way>) {
            //reads bincode file, values are used to make RoadNetwork
            let objs = bincode::deserialize::<ExportOsm>(data).unwrap();

            let mut nodes = HashMap::new();
            let ways = objs.ways;

            let mut new_ways = vec![];

            for node in objs.nodes {
                nodes.insert(
                    node.id.0,
                    Node {
                        id: node.id.0,
                        lat: (node.lat() * f64::powi(10.0, 7)) as i64,
                        lon: (node.lon() * f64::powi(10.0, 7)) as i64,
                    },
                );
            }

            for way in ways {
                if let Some((key, road_type)) = way.tags.iter().find(|(k, _)| k.eq(&"highway")) {
                    if let Some(speed) = speed_calc(road_type.as_str()) {
                        new_ways.push(Way {
                            id: way.id.0,
                            speed,
                            refs: way.nodes.into_iter().map(|x| x.0).collect(),
                        });
                    }
                } else if let Some((key, road_type)) = way.tags.iter().find(|(k, _)| k.eq(&"foot"))
                {
                    new_ways.push(Way {
                        id: way.id.0,
                        speed: 4,
                        refs: way.nodes.into_iter().map(|x| x.0).collect(),
                    });
                }
            }

            (nodes, new_ways)
        }

        pub fn read_from_osm_file(path: &str) -> Option<(HashMap<i64, Node>, Vec<Way>)> {
            //reads osm.pbf file, values are used to make RoadNetwork
            let mut nodes = HashMap::new();
            let mut ways = Vec::new();
            let path_cleaned = std::path::Path::new(&path);
            let r = std::fs::File::open(path_cleaned).unwrap();
            let mut reader = osmpbfreader::OsmPbfReader::new(r);
            for obj in reader.iter().map(Result::unwrap) {
                match obj {
                    OsmObj::Node(e) => {
                        nodes.insert(
                            e.id.0,
                            Node {
                                id: e.id.0,
                                lat: (e.lat() * f64::powi(10.0, 7)) as i64,
                                lon: (e.lon() * f64::powi(10.0, 7)) as i64,
                            },
                        );
                    }
                    OsmObj::Way(e) => {
                        if let Some((key, road_type)) =
                            e.tags.iter().find(|(k, _)| k.eq(&"highway"))
                        {
                            if let Some(speed) = speed_calc(road_type.as_str()) {
                                ways.push(Way {
                                    id: e.id.0,
                                    speed,
                                    refs: e.nodes.into_iter().map(|x| x.0).collect(),
                                });
                            }
                        }
                    }
                    _ => {}
                }
            }
            Some((nodes, ways))
        }

        pub fn reduce_to_largest_connected_component(self) -> Self {
            //reduces graph to largest connected component through nodes visited with dijkstra
            let mut counter = 0;
            let mut number_times_node_visted: HashMap<i64, i32> = HashMap::new();
            let mut shortest_path_graph = RoadDijkstra::new(&self);
            let mut max_connections = 0;

            while let Some(source_id) =
                shortest_path_graph.get_unvisted_node_id(&number_times_node_visted)
            {
                counter += 1;
                let mut shortest_path_graph = RoadDijkstra::new(&self);
                shortest_path_graph.dijkstra(source_id, -1);
                for node in shortest_path_graph.visited_nodes.keys() {
                    number_times_node_visted.insert(*node, counter);
                }
                if number_times_node_visted.len() > (self.nodes.len() / 2) {
                    break;
                }
            }
            let mut new_node_list = Vec::new();
            new_node_list = number_times_node_visted.iter().collect();
            new_node_list.sort_by(|(node1, counter1), (node2, counter2)| counter1.cmp(counter2));

            let connected_components = &mut new_node_list
                .chunk_by(|(node1, counter1), (node2, counter2)| counter1 == counter2);

            let mut largest_node_set = Vec::new();
            let mut prev_set_size = 0;

            for node_set in connected_components.by_ref() {
                if node_set.len() > prev_set_size {
                    largest_node_set = node_set.to_vec();
                    prev_set_size = node_set.len();
                }
            }

            let lcc_nodes = largest_node_set
                .iter()
                .map(|(id, _)| (**id, *self.nodes.get(id).unwrap()))
                .collect::<HashMap<i64, Node>>();

            RoadNetwork::new(lcc_nodes, self.raw_ways)
        }
    }
}
