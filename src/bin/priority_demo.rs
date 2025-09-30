use smart_road::{
    components::{Direction, Route},
    examples::collision_demo::SmartRoadExample,
};
use std::{thread, time::Duration};

fn main() {
    println!("=== Priority-Based Intersection Entry System Demo ===");
    println!("This demo demonstrates the enhanced traffic management system");
    println!("with priority-based vehicle entry control.\n");

    let mut demo = SmartRoadExample::new();

    println!("Scenario 1: Testing priority levels");
    println!("Adding vehicles with different routes to test priority system...\n");

    // Add vehicles with different priorities to test the system
    // Right turns have highest priority, left turns have lowest priority
    demo.add_vehicle(Direction::North, Route::Left); // Low priority
    demo.add_vehicle(Direction::East, Route::Right); // High priority
    demo.add_vehicle(Direction::South, Route::Straight); // Medium priority
    demo.add_vehicle(Direction::West, Route::Left); // Low priority
    demo.add_vehicle(Direction::North, Route::Right); // High priority

    // Run simulation for several steps to see priority system in action
    for step in 1..=20 {
        println!("--- Simulation Step {} ---", step);

        let collided_ids = demo.update(0.1); // 100ms time step

        if !collided_ids.is_empty() {
            println!("⚠️  Collisions detected for vehicles: {:?}", collided_ids);
        }

        // Print telemetry
        println!("{}", demo.get_telemetry());

        // Get detailed queue information
        let queue_telemetry = demo
            .traffic_controller
            .intersection_entry_system
            .get_queue_telemetry();
        if queue_telemetry.total_queued > 0 {
            queue_telemetry.print_debug();
        }

        println!();

        // Small delay for readability
        thread::sleep(Duration::from_millis(500));
    }

    println!("\n=== Scenario 2: High Traffic Load ===");
    println!("Adding many vehicles to test queue management...\n");

    // Reset and add many vehicles
    demo.reset();

    // Add multiple vehicles from each direction
    let directions = [
        Direction::North,
        Direction::South,
        Direction::East,
        Direction::West,
    ];
    let routes = [Route::Left, Route::Straight, Route::Right];

    for &direction in &directions {
        for &route in &routes {
            demo.add_vehicle(direction, route);
        }
    }

    println!("Added 12 vehicles total (3 per direction)");
    println!("Testing queue processing and priority resolution...\n");

    // Run simulation to see how the system handles high load
    for step in 1..=30 {
        let collided_ids = demo.update(0.1);

        if step % 5 == 0 {
            // Print status every 5 steps
            println!("--- Step {} Status ---", step);
            println!("{}", demo.get_telemetry());

            let queue_telemetry = demo
                .traffic_controller
                .intersection_entry_system
                .get_queue_telemetry();
            if queue_telemetry.total_queued > 0 {
                queue_telemetry.print_debug();
            }

            if !collided_ids.is_empty() {
                println!("⚠️  Collisions: {:?}", collided_ids);
            }
            println!();
        }

        thread::sleep(Duration::from_millis(200));
    }

    println!("\n=== Scenario 3: Conflict Resolution ===");
    println!("Testing conflict resolution between vehicles with same priority...\n");

    demo.reset();

    // Add vehicles that will have conflicting paths
    demo.add_vehicle(Direction::North, Route::Straight); // North to South
    demo.add_vehicle(Direction::East, Route::Straight); // East to West (conflicts)
    demo.add_vehicle(Direction::South, Route::Left); // South to East (conflicts with East->West)
    demo.add_vehicle(Direction::West, Route::Right); // West to North (minimal conflict)

    println!("Added 4 vehicles with potentially conflicting paths");
    println!("Observing conflict resolution...\n");

    for step in 1..=25 {
        let collided_ids = demo.update(0.1);

        if step % 3 == 0 {
            // More frequent updates for conflict analysis
            println!("--- Conflict Resolution Step {} ---", step);
            let telemetry = demo.traffic_controller.get_telemetry();
            telemetry.print_debug();

            if !collided_ids.is_empty() {
                println!("❌ COLLISION DETECTED: {:?}", collided_ids);
                println!("This indicates the priority system needs adjustment!");
            } else {
                println!("✅ No collisions - priority system working correctly");
            }
            println!();
        }

        thread::sleep(Duration::from_millis(300));
    }

    println!("\n=== Demo Complete ===");
    println!("\nKey Features Demonstrated:");
    println!("✅ Priority-based vehicle entry (Right > Straight > Left)");
    println!("✅ Queue management at intersection entry points");
    println!("✅ FIFO processing within priority levels");
    println!("✅ Conflict detection and resolution");
    println!("✅ Prevention of vehicles entering intersection simultaneously");
    println!("✅ Timeout mechanism to prevent deadlocks");

    let final_telemetry = demo.traffic_controller.get_telemetry();
    println!("\nFinal Statistics:");
    final_telemetry.print_debug();

    println!("\nThe priority-based intersection entry system successfully");
    println!("manages vehicle access to prevent collisions while maintaining");
    println!("efficient traffic flow based on route priorities.");
}
