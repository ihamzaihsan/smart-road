use crate::{
    components::{Direction, Route, path_vehicle::PathVehicle},
    systems::{
        conflict_system::Path,
        traffic_controller::TrafficController,
    },
    resources::stats::Stats,
};
use nalgebra::Vector2;

/// Example integration showing how to use the new collision system
pub struct SmartRoadExample {
    pub traffic_controller: TrafficController,
    pub stats: Stats,
    next_vehicle_id: u64,
}

impl SmartRoadExample {
    pub fn new() -> Self {
        // Create intersection at (400, 300) with 200x200 size
        let intersection_center = Vector2::new(400.0, 300.0);
        let intersection_size = 200.0;
        
        Self {
            traffic_controller: TrafficController::new(intersection_center, intersection_size),
            stats: Stats::default(),
            next_vehicle_id: 1,
        }
    }

    /// Create a sample vehicle with a path through the intersection
    pub fn create_sample_vehicle(&mut self, approach: Direction, route: Route) -> PathVehicle {
        let vehicle_id = self.next_vehicle_id;
        self.next_vehicle_id += 1;

        // Create a sample path based on approach and route
        let path_waypoints = self.generate_path_for_route(approach, route);
        let path = Path::new(path_waypoints);
        
        // Starting position based on approach
        let start_position = match approach {
            Direction::North => Vector2::new(400.0, 100.0),  // Coming from south, going north
            Direction::South => Vector2::new(400.0, 500.0),  // Coming from north, going south  
            Direction::East => Vector2::new(100.0, 300.0),   // Coming from west, going east
            Direction::West => Vector2::new(700.0, 300.0),   // Coming from east, going west
        };

        PathVehicle::new(
            vehicle_id,
            start_position,
            approach,
            route,
            path,
            0, // texture index
        )
    }

    /// Generate waypoints for a given route
    fn generate_path_for_route(&self, approach: Direction, route: Route) -> Vec<Vector2<f32>> {
        let intersection_center = Vector2::new(400.0, 300.0);
        let approach_distance = 200.0;
        let exit_distance = 200.0;

        match (approach, route) {
            // North approach (coming from south)
            (Direction::North, Route::Straight) => vec![
                Vector2::new(400.0, 100.0),   // Start
                Vector2::new(400.0, 200.0),   // Approach intersection
                Vector2::new(400.0, 400.0),   // Through intersection
                Vector2::new(400.0, 500.0),   // Exit
            ],
            (Direction::North, Route::Left) => vec![
                Vector2::new(400.0, 100.0),   // Start
                Vector2::new(400.0, 200.0),   // Approach intersection
                Vector2::new(380.0, 280.0),   // Begin turn
                Vector2::new(320.0, 300.0),   // Mid turn
                Vector2::new(200.0, 300.0),   // Exit
            ],
            (Direction::North, Route::Right) => vec![
                Vector2::new(400.0, 100.0),   // Start
                Vector2::new(400.0, 250.0),   // Approach intersection
                Vector2::new(420.0, 280.0),   // Begin turn
                Vector2::new(480.0, 300.0),   // Mid turn
                Vector2::new(600.0, 300.0),   // Exit
            ],

            // East approach (coming from west)
            (Direction::East, Route::Straight) => vec![
                Vector2::new(100.0, 300.0),   // Start
                Vector2::new(200.0, 300.0),   // Approach intersection
                Vector2::new(400.0, 300.0),   // Through intersection
                Vector2::new(600.0, 300.0),   // Exit
            ],
            (Direction::East, Route::Left) => vec![
                Vector2::new(100.0, 300.0),   // Start
                Vector2::new(200.0, 300.0),   // Approach intersection
                Vector2::new(320.0, 280.0),   // Begin turn
                Vector2::new(400.0, 200.0),   // Mid turn
                Vector2::new(400.0, 100.0),   // Exit (north)
            ],
            (Direction::East, Route::Right) => vec![
                Vector2::new(100.0, 300.0),   // Start
                Vector2::new(300.0, 300.0),   // Approach intersection
                Vector2::new(320.0, 320.0),   // Begin turn
                Vector2::new(400.0, 400.0),   // Mid turn
                Vector2::new(400.0, 500.0),   // Exit (south)
            ],

            // Add similar patterns for South and West approaches...
            _ => vec![
                intersection_center - Vector2::new(0.0, approach_distance),
                intersection_center,
                intersection_center + Vector2::new(0.0, exit_distance),
            ],
        }
    }

    /// Run one simulation step
    pub fn update(&mut self, delta_time: f32) -> Vec<u64> {
        self.traffic_controller.update(delta_time, &mut self.stats)
    }

    /// Add a vehicle to the simulation
    pub fn add_vehicle(&mut self, approach: Direction, route: Route) {
        let vehicle = self.create_sample_vehicle(approach, route);
        self.traffic_controller.add_vehicle(vehicle);
    }

    /// Get telemetry for debugging
    pub fn get_telemetry(&self) -> String {
        let telemetry = self.traffic_controller.get_telemetry();
        format!(
            "Vehicles: {}, Moving: {}, Yielding: {}, Collisions: {}",
            telemetry.total_vehicles,
            telemetry.moving_vehicles, 
            telemetry.yielding_vehicles,
            self.stats.collisions
        )
    }

    /// Reset the simulation
    pub fn reset(&mut self) {
        self.traffic_controller.reset();
        self.stats = Stats::default();
        self.next_vehicle_id = 1;
    }
}

impl Default for SmartRoadExample {
    fn default() -> Self {
        Self::new()
    }
}

/// Example usage of the enhanced collision system
pub fn run_collision_test() {
    let mut simulation = SmartRoadExample::new();
    
    // Add some test vehicles
    simulation.add_vehicle(Direction::North, Route::Straight);
    simulation.add_vehicle(Direction::East, Route::Straight);  // This should create a crossing conflict
    simulation.add_vehicle(Direction::North, Route::Left);
    simulation.add_vehicle(Direction::West, Route::Right);

    println!("Starting enhanced collision system test...");
    println!("Initial state: {}", simulation.get_telemetry());

    // Run simulation for 10 seconds
    let delta_time = 1.0 / 60.0; // 60 FPS
    let total_steps = (10.0 / delta_time) as i32;

    for step in 0..total_steps {
        let collided_vehicles = simulation.update(delta_time);
        
        if !collided_vehicles.is_empty() {
            println!("Step {}: Collision detected! Vehicles: {:?}", step, collided_vehicles);
        }

        // Print telemetry every second
        if step % 60 == 0 {
            println!("Step {}: {}", step, simulation.get_telemetry());
        }

        // Add new vehicles occasionally to test the system
        if step % 180 == 0 && step > 0 {
            simulation.add_vehicle(Direction::South, Route::Straight);
        }
    }

    println!("Test completed. Final stats: {}", simulation.get_telemetry());
    println!("Key improvements demonstrated:");
    println!("- Conflict-cell based reservations instead of simple grid");
    println!("- Priority-based yielding (Right > Straight > Left)"); 
    println!("- Path-parameter based ETA calculations");
    println!("- Capsule collision detection for crossing vehicles");
    println!("- Hysteresis-based speed control");
}
