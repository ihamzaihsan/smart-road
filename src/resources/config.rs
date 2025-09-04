#[derive(Debug, Clone)]
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
            num_lanes: 3,
        }
    }
}

#[derive(Debug, Clone)]
pub struct SimulationConfig {
    pub max_vehicles: usize,
    pub spawn_cooldown: f64,
    pub cleanup_distance: f32,
    pub intersection_size: f32,
    pub detection_distance: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            max_vehicles: 20,
            spawn_cooldown: 3.0,
            cleanup_distance: 350.0,
            intersection_size: 300.0,
            detection_distance: 200.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PhysicsConfig {
    pub max_vehicle_speed: f32,
    pub acceleration: f32,
    pub deceleration: f32,
    pub min_safety_distance: f32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            max_vehicle_speed: 200.0,
            acceleration: 200.0,
            deceleration: 75.0,
            min_safety_distance: 50.0,
        }
    }
}