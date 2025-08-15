use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    pub id: u64,
    pub route: crate::types::Route,
    pub velocity: f32,
    pub distance_remaining: f32,
}
