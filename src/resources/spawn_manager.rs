use bevy::prelude::*;
use crate::components::{Direction, Route};
use std::collections::HashMap;
use rand::Rng;


#[derive(Resource, Debug)]
pub struct SpawnManager {
    pub next_id: u64,
    pub last_spawn_time: HashMap<Direction, f64>,
    pub spawn_cooldown: f64,
    pub spawn_distance: f32,
    pub min_spawn_separation: f32,
}

impl Default for SpawnManager {
    fn default() -> Self {
        Self {
            next_id: 1,
            last_spawn_time: HashMap::new(),
            spawn_cooldown: 0.5, 
            spawn_distance: 300.0, 
            min_spawn_separation: 60.0, 
        }
    }
}

impl SpawnManager {
    pub fn next_vehicle_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Check if enough time has passed since last spawn in this direction
    pub fn can_spawn(&self, direction: Direction, current_time: f64) -> bool {
        match self.last_spawn_time.get(&direction) {
            Some(&last_time) => current_time - last_time >= self.spawn_cooldown,
            None => true,
        }
    }

    /// Record a spawn time for a direction
    pub fn record_spawn(&mut self, direction: Direction, time: f64) {
        self.last_spawn_time.insert(direction, time);
    }

    /// Calculate spawn position for a vehicle based on direction and lane
    pub fn get_spawn_position(&self, direction: Direction, lane_offset: f32) -> Vec3 {
        match direction {
            Direction::North => Vec3::new(lane_offset, -self.spawn_distance, 5.0),
            Direction::South => Vec3::new(lane_offset, self.spawn_distance, 5.0),
            Direction::East => Vec3::new(-self.spawn_distance, lane_offset, 5.0),
            Direction::West => Vec3::new(self.spawn_distance, lane_offset, 5.0),
        }
    }

    /// Get lane centers for a given direction based on road configuration
    pub fn get_lane_centers(&self, direction: Direction, road_config: &crate::resources::RoadConfig) -> Vec<f32> {
        let lane_width = road_config.lane_width;
        let lanes_per_direction = road_config.num_lanes;
        let total_lanes = lanes_per_direction * 2;
        let road_width = lane_width * total_lanes as f32;
        let left_edge = -road_width / 2.0;

        // Calculate all lane centers
        let mut all_lane_centers = Vec::new();
        for idx in 0..total_lanes {
            all_lane_centers.push(left_edge + lane_width / 2.0 + idx as f32 * lane_width);
        }

        // Return appropriate lanes based on direction
        match direction {
            Direction::North | Direction::East => {
                // Use right group of lanes (vehicles moving towards positive direction)
                all_lane_centers.into_iter().skip(lanes_per_direction).collect()
            }
            Direction::South | Direction::West => {
                // Use left group of lanes (vehicles moving towards negative direction)
                all_lane_centers.into_iter().take(lanes_per_direction).collect()
            }
        }
    }

    /// Get a random lane center for a given direction
    pub fn get_random_lane_center(&self, direction: Direction, road_config: &crate::resources::RoadConfig) -> f32 {
        let lane_centers = self.get_lane_centers(direction, road_config);
        let mut rng = rand::thread_rng();
        let index = rng.gen_range(0..lane_centers.len());
        lane_centers[index]
    }

    /// Randomly select a route for a vehicle
    pub fn random_route() -> Route {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        match rng.gen_range(0..3) {
            0 => Route::Straight,
            1 => Route::Left,
            _ => Route::Right,
        }
    }
}

/// Resource that manages automatic vehicle spawning with configurable timing
#[derive(Resource, Debug)]
pub struct AutoSpawnTimer {
    /// Timer for controlling spawn frequency
    pub timer: Timer,
    /// Minimum spawn interval in seconds
    pub min_interval: f32,
    /// Maximum spawn interval in seconds
    pub max_interval: f32,
    /// Whether automatic spawning is enabled
    pub enabled: bool,
}

impl Default for AutoSpawnTimer {
    fn default() -> Self {
        Self {
            timer: Timer::from_seconds(1.0, TimerMode::Once), // Fixed 1-second initial delay
            min_interval: 1.0,  // Fixed 1-second interval
            max_interval: 1.0,  // Fixed 1-second interval (same as min)
            enabled: true,
        }
    }
}

impl AutoSpawnTimer {
    /// Reset the timer with a fixed 1-second interval
    pub fn reset_with_fixed_interval(&mut self) {
        // Use fixed 1-second interval (min_interval = max_interval = 1.0)
        self.timer = Timer::from_seconds(self.min_interval, TimerMode::Once);
    }

    /// Get a random spawn direction
    pub fn random_direction() -> Direction {
        let mut rng = rand::thread_rng();
        match rng.gen_range(0..4) {
            0 => Direction::North,
            1 => Direction::South,
            2 => Direction::East,
            _ => Direction::West,
        }
    }
}
