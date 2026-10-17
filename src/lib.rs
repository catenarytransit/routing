// Copyright Chelsea Wen
// Cleaned up somewhat by Kyler Chin
pub mod raptor;
pub mod road_dijkstras;
pub mod road_network;
pub mod timetable;
//pub mod transfers;
//pub mod transit_dijkstras;
//pub mod transit_network;

use serde::{Deserialize, Serialize};

pub static TEN_TO_14: f64 = 100000000000000.0;

#[derive(PartialEq, Eq, PartialOrd, Ord, Copy, Clone, Debug, Hash, Serialize, Deserialize)]
pub enum NodeType {
    Untyped = 0,
    Arrival = 1,
    Transfer = 2,
    Departure = 3,
}

impl From<String> for NodeType {
    fn from(read_val: String) -> Self {
        match read_val.as_str() {
            "Untyped" => NodeType::Untyped,
            "Arrival" => NodeType::Arrival,
            "Transfer" => NodeType::Transfer,
            "Departure" => NodeType::Departure,
            &_ => NodeType::Untyped,
        }
    }
}

impl From<NodeType> for String {
    fn from(v: NodeType) -> String {
        match v {
            NodeType::Untyped => "Untyped".to_string(),
            NodeType::Arrival => "Arrival".to_string(),
            NodeType::Transfer => "Transfer".to_string(),
            NodeType::Departure => "Departure".to_string(),
        }
    }
}

pub fn coord_to_int(x: f64, y: f64) -> (i64, i64) {
    let x = (x * TEN_TO_14) as i64;
    let y = (y * TEN_TO_14) as i64;
    (x, y)
}
pub fn int_to_coord(x: i64, y: i64) -> (f64, f64) {
    let x = x as f64 / TEN_TO_14;
    let y = y as f64 / TEN_TO_14;
    (x, y)
}
