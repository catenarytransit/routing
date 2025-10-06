// Copyright Chelsea Wen
// Cleaned up somewhat by Kyler Chin
#![deny(
    clippy::mutable_key_type,
    clippy::map_entry,
    clippy::boxed_local,
    clippy::let_unit_value,
    clippy::redundant_allocation,
    clippy::bool_comparison,
    clippy::bind_instead_of_map,
    clippy::vec_box,
    clippy::while_let_loop,
    clippy::useless_asref,
    clippy::repeat_once,
    clippy::deref_addrof,
    clippy::suspicious_map,
    clippy::arc_with_non_send_sync,
    clippy::single_char_pattern,
    clippy::for_kv_map,
    clippy::let_and_return,
    clippy::iter_nth,
    clippy::iter_cloned_collect,
    clippy::bytes_nth,
    clippy::deprecated_clippy_cfg_attr,
    clippy::match_result_ok,
    clippy::cmp_owned,
    clippy::cmp_null,
    clippy::op_ref,
    clippy::useless_vec,
    clippy::module_inception
)]
pub mod road_network;
pub use crate::road_network::road_graph_construction::RoadNetwork;

pub mod road_dijkstras;
pub mod transfers;
pub mod transit_dijkstras;
pub mod transit_network;

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
