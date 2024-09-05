#[derive(PartialEq, Eq, PartialOrd, Ord, Copy, Clone, Debug, Hash)]
pub enum NodeType {
    Untyped = 0,
    Arrival = 1,
    Transfer = 2,
    Departure = 3,
}
