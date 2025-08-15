/// Global configuration for road visuals and layout.
use bevy::prelude::Resource;

#[derive(Resource, Debug, Clone)]
pub struct RoadConfig {
    pub lane_width: f32,
    pub dash_length: f32,
    pub dash_gap: f32,
    pub num_lanes: usize,
}

impl Default for RoadConfig {
    fn default() -> Self {
        Self {
            lane_width: 50.0,
            dash_length: 20.0,
            dash_gap: 15.0,
            num_lanes: 2,
        }
    }
}
