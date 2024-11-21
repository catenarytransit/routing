use serde::{Deserialize, Serialize};

#[derive(PartialEq, Eq, PartialOrd, Ord, Copy, Clone, Debug, Hash, Serialize, Deserialize)]
pub enum NodeType {
    Untyped = 0,
    Arrival = 1,
    Transfer = 2,
    Departure = 3,
}
