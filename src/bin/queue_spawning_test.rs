/// Test to verify vehicles spawn behind each other in queue
/// This test checks that when multiple vehicles spawn and approach the same queue,
/// they position themselves behind each other instead of overlapping.

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
    println!("=== Queue Spawning Test ===\n");
    println!("Testing that vehicles spawn behind each other in queue...\n");

    let intersection_center = Vector2::new(400.0, 300.0);
    let intersection_size = 200.0;

    let mut controller = TrafficController::new(intersection_center, intersection_size);
    let mut stats = Stats::default();

    // Spawn 5 vehicles approaching from the same direction
    println!("Spawning 5 vehicles from North direction...");
    for i in 1..=5 {
        let mut vehicle = create_vehicle(
            i,
            Direction::North,
            Route::Straight,
            Vector2::new(400.0, 50.0 + (i as f32) * 10.0), // Staggered start positions
        );
        vehicle.speed = 50.0;
        vehicle.target_speed = 50.0;
        vehicle.state = VehicleState::Moving;
        
        controller.add_vehicle(vehicle);
        println!("  Added vehicle {} at position (400, {})", i, 50.0 + (i as f32) * 10.0);
    }

    println!("\nSimulating to let vehicles reach queue...\n");

    let delta_time = 0.1; // 100ms per step

    // Simulate for several steps to let vehicles enter queue
    for step in 1..=30 {
        let collided_ids = controller.update(delta_time, &mut stats);

        if !collided_ids.is_empty() {
            println!("❌ FAIL at Step {}: Collision detected between vehicles: {:?}", step, collided_ids);
            println!("   This means vehicles are overlapping when entering queue!");
            
            // Print vehicle positions
            println!("\n   Vehicle positions:");
            for vehicle in controller.vehicles.iter() {
                println!("     Vehicle {}: pos=({:.1}, {:.1}), state={:?}, speed={:.1}",
                    vehicle.id,
                    vehicle.position.x,
                    vehicle.position.y,
                    vehicle.state,
                    vehicle.speed
                );
            }
            
            return;
        }

        // Check queue status every 5 steps
        if step % 5 == 0 {
            let queue_telemetry = controller
                .intersection_entry_system
                .get_queue_telemetry();
            
            println!("Step {}: Queued vehicles: {}, Collisions: {}", 
                step, 
                queue_telemetry.north_queue_size,
                stats.collisions
            );

            // Print vehicle positions when vehicles are in queue
            if queue_telemetry.north_queue_size > 0 {
                let mut waiting_vehicles: Vec<_> = controller.vehicles
                    .iter()
                    .filter(|v| v.state == VehicleState::Waiting)
                    .collect();
                
                waiting_vehicles.sort_by(|a, b| b.position.y.partial_cmp(&a.position.y).unwrap());
                
                println!("  Waiting vehicles (sorted by position):");
                for vehicle in waiting_vehicles {
                    println!("    Vehicle {}: y={:.1}", vehicle.id, vehicle.position.y);
                }
                
                // Check spacing between consecutive vehicles
                let positions: Vec<f32> = controller.vehicles
                    .iter()
                    .filter(|v| v.state == VehicleState::Waiting)
                    .map(|v| v.position.y)
                    .collect();
                
                if positions.len() >= 2 {
                    let mut sorted_positions = positions.clone();
                    sorted_positions.sort_by(|a, b| b.partial_cmp(a).unwrap());
                    
                    println!("  Spacing between vehicles:");
                    for i in 0..sorted_positions.len() - 1 {
                        let spacing = sorted_positions[i] - sorted_positions[i + 1];
                        println!("    Between vehicle {} and {}: {:.1} units", i+1, i+2, spacing);
                        
                        if spacing < 50.0 {
                            println!("    ⚠️  WARNING: Spacing too small! Risk of collision.");
                        }
                    }
                }
            }
        }

        // Stop if all vehicles are queued
        let all_waiting = controller.vehicles.iter().all(|v| v.state == VehicleState::Waiting);
        if all_waiting && controller.vehicles.len() == 5 {
            println!("\n✅ SUCCESS: All 5 vehicles queued without collision!");
            println!("\nFinal vehicle positions:");
            
            let mut sorted_vehicles: Vec<_> = controller.vehicles.iter().collect();
            sorted_vehicles.sort_by(|a, b| b.position.y.partial_cmp(&a.position.y).unwrap());
            
            for vehicle in sorted_vehicles {
                println!("  Vehicle {}: ({:.1}, {:.1})", vehicle.id, vehicle.position.x, vehicle.position.y);
            }
            
            // Verify spacing
            let positions: Vec<f32> = controller.vehicles
                .iter()
                .map(|v| v.position.y)
                .collect();
            
            let mut sorted_positions = positions.clone();
            sorted_positions.sort_by(|a, b| b.partial_cmp(a).unwrap());
            
            let mut min_spacing = f32::MAX;
            for i in 0..sorted_positions.len() - 1 {
                let spacing = sorted_positions[i] - sorted_positions[i + 1];
                min_spacing = min_spacing.min(spacing);
            }
            
            println!("\nMinimum spacing: {:.1} units", min_spacing);
            
            if min_spacing >= 100.0 {
                println!("✅ PASS: Spacing is adequate (>= 100 units)");
            } else if min_spacing >= 50.0 {
                println!("⚠️  WARNING: Spacing is tight but acceptable (>= 50 units)");
            } else {
                println!("❌ FAIL: Spacing is too small (< 50 units) - vehicles too close!");
            }
            
            if stats.collisions == 0 {
                println!("\n✅ PASS: No collisions detected during queue formation!");
            }
            
            return;
        }
    }

    println!("\n⚠️  Test incomplete: Not all vehicles entered queue in 30 steps");
    println!("Final state: {} vehicles, {} queued, {} collisions",
        controller.vehicles.len(),
        controller.intersection_entry_system.get_queue_telemetry().north_queue_size,
        stats.collisions
    );
}

fn create_vehicle(id: u64, approach: Direction, route: Route, position: Vector2<f32>) -> PathVehicle {
    let start = position;
    let end = Vector2::new(400.0, 500.0); // End point north of intersection

    let path = Path::new(vec![start, end]);

    PathVehicle::new(id, start, approach, route, path, 0)
}
