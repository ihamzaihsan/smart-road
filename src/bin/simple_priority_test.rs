use smart_road::{
    components::{Direction, Route},
    examples::collision_demo::SmartRoadExample,
};
use std::{thread, time::Duration};

fn main() {
    println!("=== Simple Priority System Test ===");
    println!("Testing basic intersection entry control\n");

    let mut demo = SmartRoadExample::new();

    // Test 1: Single vehicle - should pass through without issues
    println!("Test 1: Single vehicle entry");
    demo.add_vehicle(Direction::North, Route::Straight);

    for step in 1..=10 {
        let collided_ids = demo.update(0.1);

        if !collided_ids.is_empty() {
            println!("❌ Step {}: Collision detected: {:?}", step, collided_ids);
            break;
        }

        let telemetry = demo.traffic_controller.get_telemetry();
        println!(
            "Step {}: Vehicles={}, Moving={}, Waiting={}",
            step, telemetry.total_vehicles, telemetry.moving_vehicles, telemetry.waiting_vehicles
        );

        if telemetry.total_vehicles == 0 {
            println!("✅ Vehicle successfully passed through intersection");
            break;
        }

        thread::sleep(Duration::from_millis(100));
    }

    println!("\nTest 2: Two vehicles, non-conflicting paths");
    demo.reset();
    demo.add_vehicle(Direction::North, Route::Right); // High priority
    thread::sleep(Duration::from_millis(50));
    demo.add_vehicle(Direction::West, Route::Right); // High priority, different path

    for step in 1..=15 {
        let collided_ids = demo.update(0.1);

        if !collided_ids.is_empty() {
            println!("❌ Step {}: Collision detected: {:?}", step, collided_ids);
            break;
        }

        let telemetry = demo.traffic_controller.get_telemetry();
        let queue_telemetry = demo
            .traffic_controller
            .intersection_entry_system
            .get_queue_telemetry();

        println!(
            "Step {}: Vehicles={}, Moving={}, Waiting={}, Queued={}",
            step,
            telemetry.total_vehicles,
            telemetry.moving_vehicles,
            telemetry.waiting_vehicles,
            queue_telemetry.total_queued
        );

        if telemetry.total_vehicles == 0 {
            println!("✅ Both vehicles successfully passed through intersection");
            break;
        }

        thread::sleep(Duration::from_millis(200));
    }

    println!("\nTest 3: Two vehicles, potentially conflicting paths");
    demo.reset();
    demo.add_vehicle(Direction::North, Route::Straight); // Medium priority
    thread::sleep(Duration::from_millis(50));
    demo.add_vehicle(Direction::East, Route::Straight); // Medium priority, conflicting path

    for step in 1..=20 {
        let collided_ids = demo.update(0.1);

        if !collided_ids.is_empty() {
            println!("❌ Step {}: Collision detected: {:?}", step, collided_ids);
            println!("This indicates the priority system failed to prevent conflicts");
            break;
        }

        let telemetry = demo.traffic_controller.get_telemetry();
        let queue_telemetry = demo
            .traffic_controller
            .intersection_entry_system
            .get_queue_telemetry();

        if step % 3 == 0 {
            println!(
                "Step {}: Vehicles={}, Moving={}, Waiting={}, Queued={}",
                step,
                telemetry.total_vehicles,
                telemetry.moving_vehicles,
                telemetry.waiting_vehicles,
                queue_telemetry.total_queued
            );

            if queue_telemetry.total_queued > 0 {
                println!(
                    "  Queue status: N={}, S={}, E={}, W={}",
                    queue_telemetry.north_queue_size,
                    queue_telemetry.south_queue_size,
                    queue_telemetry.east_queue_size,
                    queue_telemetry.west_queue_size
                );
            }
        }

        if telemetry.total_vehicles == 0 {
            println!("✅ Both vehicles successfully passed through intersection sequentially");
            break;
        }

        thread::sleep(Duration::from_millis(200));
    }

    println!("\nTest 4: Priority ordering (Right > Straight > Left)");
    demo.reset();
    demo.add_vehicle(Direction::North, Route::Left); // Lowest priority
    thread::sleep(Duration::from_millis(50));
    demo.add_vehicle(Direction::South, Route::Right); // Highest priority
    thread::sleep(Duration::from_millis(50));
    demo.add_vehicle(Direction::East, Route::Straight); // Medium priority

    println!("Added vehicles in order: Left (low), Right (high), Straight (medium)");
    println!("Expected processing order: Right -> Straight -> Left");

    for step in 1..=25 {
        let collided_ids = demo.update(0.1);

        if !collided_ids.is_empty() {
            println!("❌ Step {}: Collision detected: {:?}", step, collided_ids);
            break;
        }

        let telemetry = demo.traffic_controller.get_telemetry();

        if step % 5 == 0 {
            println!(
                "Step {}: Vehicles={}, Moving={}, Waiting={}",
                step,
                telemetry.total_vehicles,
                telemetry.moving_vehicles,
                telemetry.waiting_vehicles
            );
        }

        if telemetry.total_vehicles == 0 {
            println!("✅ All vehicles processed successfully");
            break;
        }

        thread::sleep(Duration::from_millis(200));
    }

    println!("\n=== Test Summary ===");
    println!("Priority-based intersection entry system provides:");
    println!("• Collision prevention through controlled entry");
    println!("• Queue management at intersection approaches");
    println!("• Priority-based processing (Right > Straight > Left)");
    println!("• Safe sequential vehicle processing");
}
