/// Scenario test: verify vehicles maintain movement when higher priority respawn occurs
use nalgebra::Vector2;
use smart_road::{
    components::{path_vehicle::PathVehicle, Direction, Route, VehicleState},
    resources::stats::Stats,
    systems::{
        conflict_system::Path,
        traffic_controller::TrafficController,
    },
};

fn main() {
    println!("=== Priority Respawn Interaction Test ===\n");

    let intersection_center = Vector2::new(400.0, 300.0);
    let intersection_size = 200.0;
    let mut controller = TrafficController::new(intersection_center, intersection_size);
    let mut stats = Stats::default();

    // Spawn initial vehicle from North going straight (lower priority)
    let mut north_vehicle = create_vehicle(
        1,
        Direction::North,
        Route::Straight,
        Vector2::new(400.0, 120.0),
    );
    north_vehicle.speed = 80.0;
    north_vehicle.target_speed = 80.0;
    north_vehicle.state = VehicleState::Moving;
    controller.add_vehicle(north_vehicle);

    println!("Step 0: Added northbound straight vehicle (ID 1)\n");

    // Simulate a few steps until the vehicle approaches the queue and is granted access
    simulate_steps(&mut controller, &mut stats, 5, 0.1, "Pre-respawn phase");

    // Spawn higher-priority vehicle from East turning right
    let mut east_vehicle = create_vehicle(
        2,
        Direction::East,
        Route::Right,
        Vector2::new(120.0, 300.0),
    );
    east_vehicle.speed = 80.0;
    east_vehicle.target_speed = 80.0;
    east_vehicle.state = VehicleState::Moving;
    controller.add_vehicle(east_vehicle);

    println!("\nSpawned eastbound right-turn vehicle (ID 2) with higher priority\n");

    // Continue simulation to watch for regressions when higher priority vehicle arrives
    simulate_steps(
        &mut controller,
        &mut stats,
        12,
        0.1,
        "Post-respawn phase",
    );

    println!("\n=== Test complete ===");
}

fn simulate_steps(
    controller: &mut TrafficController,
    stats: &mut Stats,
    steps: usize,
    delta_time: f32,
    phase: &str,
) {
    println!("-- {} --", phase);
    for step in 1..=steps {
        let collisions = controller.update(delta_time, stats);

        if !collisions.is_empty() {
            println!("  ❌ Collision detected on step {}: {:?}", step, collisions);
        }

        println!("  Step {}:", step);
        for vehicle in &controller.vehicles {
            println!(
                "    Vehicle {} -> pos=({:.1}, {:.1}), state={:?}, speed={:.1}",
                vehicle.id, vehicle.position.x, vehicle.position.y, vehicle.state, vehicle.speed
            );
        }
    }
}

fn create_vehicle(
    id: u64,
    approach: Direction,
    route: Route,
    start: Vector2<f32>,
) -> PathVehicle {
    // Basic straight-line path for debugging scenarios
    let end = match approach {
        Direction::North => Vector2::new(400.0, 500.0),
        Direction::South => Vector2::new(400.0, 100.0),
        Direction::East => Vector2::new(700.0, 300.0),
        Direction::West => Vector2::new(100.0, 300.0),
    };

    let path = Path::new(vec![start, end]);
    PathVehicle::new(id, start, approach, route, path, 0)
}
