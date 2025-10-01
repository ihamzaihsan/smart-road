/// Test to verify rear-end collision detection in queues
/// This test specifically checks that when a moving vehicle hits a waiting vehicle in queue,
/// it is properly detected as a collision.

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
    println!("=== Queue Rear-End Collision Detection Test ===\n");

    let intersection_center = Vector2::new(400.0, 300.0);
    let intersection_size = 200.0;

    let mut controller = TrafficController::new(intersection_center, intersection_size);
    let mut stats = Stats::default();

    // Create a queued vehicle (already waiting)
    let waiting_vehicle = create_vehicle(
        1,
        Direction::North,
        Route::Straight,
        Vector2::new(400.0, 140.0), // At the waiting line
    );

    // Create a moving vehicle approaching from behind
    let mut moving_vehicle = create_vehicle(
        2,
        Direction::North,
        Route::Straight,
        Vector2::new(400.0, 50.0), // Behind the waiting vehicle
    );
    moving_vehicle.speed = 60.0; // Moving at 60 m/s
    moving_vehicle.target_speed = 60.0;
    moving_vehicle.state = VehicleState::Moving;

    controller.add_vehicle(waiting_vehicle);
    controller.add_vehicle(moving_vehicle);

    println!("Test Setup:");
    println!("- Vehicle 1: Waiting at queue position (400, 140)");
    println!("- Vehicle 2: Moving from behind at (400, 50) with speed 60 m/s");
    println!("\nExpected: Vehicle 2 should collide with Vehicle 1 as it approaches\n");

    let delta_time = 0.1; // 100ms per step

    // Simulate for several steps
    for step in 1..=20 {
        let collided_ids = controller.update(delta_time, &mut stats);

        let remaining_vehicles = controller.vehicles.len();
        
        if remaining_vehicles >= 2 {
            println!("Step {}: Collisions = {}, Vehicle 2 position: ({:.1}, {:.1}), speed: {:.1}",
                step,
                stats.collisions,
                controller.vehicles[1].position.x,
                controller.vehicles[1].position.y,
                controller.vehicles[1].speed
            );
        } else {
            println!("Step {}: Collisions = {}, Remaining vehicles: {}",
                step,
                stats.collisions,
                remaining_vehicles
            );
        }

        if !collided_ids.is_empty() {
            println!("\n✅ SUCCESS: Collision detected!");
            println!("   Collided vehicles: {:?}", collided_ids);
            println!("   Total collisions in stats: {}", stats.collisions);
            println!("   Vehicles were removed from simulation (as expected for collisions)");
            
            // Verify collision was properly counted
            if stats.collisions > 0 {
                println!("\n✅ PASS: Rear-end collision in queue was properly detected and counted!");
                return;
            }
        }

        // If both vehicles were removed, collision was detected
        if remaining_vehicles < 2 && stats.collisions > 0 {
            println!("\n✅ SUCCESS: Both vehicles removed due to collision!");
            println!("\n✅ PASS: Rear-end collision in queue was properly detected and counted!");
            return;
        }

        // Check if vehicle 2 got too close without collision being detected
        if remaining_vehicles >= 2 {
            let distance = (controller.vehicles[1].position - controller.vehicles[0].position).magnitude();
            if distance < 50.0 && stats.collisions == 0 {
                println!("   WARNING: Vehicle 2 is very close ({}m) to vehicle 1 but no collision detected yet", distance);
            }

            // Stop if vehicle passed through
            if controller.vehicles[1].position.y > controller.vehicles[0].position.y {
                println!("\n❌ FAIL: Vehicle 2 passed through vehicle 1 without collision detection!");
                println!("   This means rear-end collisions in queue are NOT being detected properly.");
                break;
            }
        }
    }

    if stats.collisions == 0 {
        println!("\n❌ FAIL: No collisions detected after 20 steps!");
        println!("   Rear-end collision detection in queues is NOT working.");
    }
}

fn create_vehicle(id: u64, approach: Direction, route: Route, position: Vector2<f32>) -> PathVehicle {
    let start = position;
    let end = Vector2::new(400.0, 500.0); // End point north of intersection

    let path = Path::new(vec![start, end]);

    let mut vehicle = PathVehicle::new(id, start, approach, route, path, 0);
    vehicle.state = VehicleState::Waiting;
    vehicle.speed = 0.0;
    vehicle.target_speed = 0.0;
    vehicle
}
