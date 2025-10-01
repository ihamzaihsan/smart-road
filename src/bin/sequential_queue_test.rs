/// Simple test: spawn vehicles one at a time and verify they queue properly
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
    println!("=== Sequential Queue Entry Test ===\n");
    println!("This test adds vehicles one at a time to verify proper queue positioning.\n");

    let intersection_center = Vector2::new(400.0, 300.0);
    let intersection_size = 200.0;
    let mut controller = TrafficController::new(intersection_center, intersection_size);
    let mut stats = Stats::default();

    // Add first vehicle close to queue position
    println!("Adding vehicle 1 close to waiting position...");
    let mut v1 = create_vehicle(1, Direction::North, Route::Straight, Vector2::new(400.0, 150.0));
    v1.speed = 20.0;
    v1.target_speed = 20.0;
    v1.state = VehicleState::Moving;
    controller.add_vehicle(v1);

    // Run a few steps to let it enter queue
    for i in 1..=3 {
        println!("\nStep {}: Running update...", i);
        let collisions = controller.update(0.1, &mut stats);
        
        if !collisions.is_empty() {
            println!("  ❌ Collision detected: {:?}", collisions);
        }
        
        let v1_state = controller.vehicles.iter().find(|v| v.id == 1);
        if let Some(v) = v1_state {
            println!("  Vehicle 1: pos=({:.1}, {:.1}), state={:?}, speed={:.1}",
                v.position.x, v.position.y, v.state, v.speed);
        }
    }

    println!("\n--- Vehicle 1 should now be in queue ---\n");

    // Now add second vehicle
    println!("Adding vehicle 2 close to waiting position...");
    let mut v2 = create_vehicle(2, Direction::North, Route::Straight, Vector2::new(400.0, 150.0));
    v2.speed = 20.0;
    v2.target_speed = 20.0;
    v2.state = VehicleState::Moving;
    controller.add_vehicle(v2);

    println!("Vehicle 2 added at same spawn position as vehicle 1 was.\n");

    // Run more steps
    for i in 4..=8 {
        println!("\nStep {}: Running update...", i);
        let collisions = controller.update(0.1, &mut stats);
        
        if !collisions.is_empty() {
            println!("  ❌ FAIL: Collision detected: {:?}", collisions);
            println!("  Vehicle 2 should have been positioned behind vehicle 1!");
            break;
        }
        
        println!("  Vehicles:");
        for v in &controller.vehicles {
            println!("    Vehicle {}: pos=({:.1}, {:.1}), state={:?}",
                v.id, v.position.x, v.position.y, v.state);
        }
        
        // Check if both are waiting
        let both_waiting = controller.vehicles.iter()
            .all(|v| v.state == VehicleState::Waiting);
        
        if both_waiting && controller.vehicles.len() == 2 {
            println!("\n✅ Both vehicles are now waiting in queue!");
            
            let v1_pos = controller.vehicles.iter().find(|v| v.id == 1).unwrap().position.y;
            let v2_pos = controller.vehicles.iter().find(|v| v.id == 2).unwrap().position.y;
            let spacing = (v1_pos - v2_pos).abs();
            
            println!("  Vehicle 1 at y={:.1}", v1_pos);
            println!("  Vehicle 2 at y={:.1}", v2_pos);
            println!("  Spacing: {:.1} units", spacing);
            
            if spacing >= 100.0 {
                println!("\n✅ PASS: Vehicles properly spaced! No collision.");
            } else {
                println!("\n⚠️  WARNING: Spacing is less than 100 units.");
            }
            
            if stats.collisions == 0 {
                println!("✅ PASS: Zero collisions during queue entry!");
            } else {
                println!("❌ FAIL: {} collision(s) detected", stats.collisions);
            }
            
            return;
        }
    }

    if stats.collisions > 0 {
        println!("\n❌ FAIL: Collisions occurred during queue entry");
    }
}

fn create_vehicle(id: u64, approach: Direction, route: Route, position: Vector2<f32>) -> PathVehicle {
    let start = position;
    let end = Vector2::new(400.0, 500.0);
    let path = Path::new(vec![start, end]);
    PathVehicle::new(id, start, approach, route, path, 0)
}
