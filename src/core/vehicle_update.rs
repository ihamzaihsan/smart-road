use crate::components::{Vehicle, VehicleState};
use crate::core::physics::{
    quantize_with_hysteresis, speed_tier_from_distance, ConflictCalendar, VehiclePhysics,
    SPEED_LEVELS,
};
use crate::resources::{RoadConfig, StatsManager};
use nalgebra::Vector2;
use std::fs::OpenOptions;
use std::io::Write;

/// Debug logging helper
fn log_debug(message: &str) {
    if let Ok(mut file) = OpenOptions::new()
        .create(true)
        .append(true)
        .open("logs/collision_debug.log")
    {
        let _ = writeln!(file, "{}", message);
    }
}

/// Emergency separation: Push overlapping vehicles apart to prevent collision lock
/// EXCEPTION: Right-turn vehicles are excluded (they use dedicated lanes)
fn separate_overlapping_vehicles(vehicles: &mut [Vehicle], delta_time: f32) {
    use crate::components::Route;
    
    let collision_threshold = 4.0; // Same as vehicle length
    let separation_force = 100.0; // pixels per second to push apart (increased from 20.0)
    
    for i in 0..vehicles.len() {
        // Skip if this is a right-turn vehicle
        let v1_is_right_turn = vehicles[i].state == VehicleState::AlwaysMoving 
            && vehicles[i].route == Route::Right;
        
        for j in (i + 1)..vehicles.len() {
            // Skip if either vehicle is a right-turn vehicle
            let v2_is_right_turn = vehicles[j].state == VehicleState::AlwaysMoving 
                && vehicles[j].route == Route::Right;
            
            if v1_is_right_turn || v2_is_right_turn {
                continue; // Right-turn vehicles use dedicated lanes, no separation needed
            }
            
            let distance = (vehicles[i].position - vehicles[j].position).magnitude();
            
            // If actually overlapping (not just within safety margin)
            if distance < collision_threshold {
                let v1_id = vehicles[i].id;
                let v2_id = vehicles[j].id;
                let v1_speed = vehicles[i].speed;
                let v2_speed = vehicles[j].speed;
                
                // Calculate separation vector (push away from each other)
                let direction = if distance > 0.01 {
                    (vehicles[j].position - vehicles[i].position) / distance
                } else {
                    // If exactly on top of each other, use random direction
                    Vector2::new(1.0, 0.0)
                };
                
                // Determine which vehicle to move based on speed
                // Move the faster one backward
                if v1_speed > v2_speed {
                    let push_distance = separation_force * delta_time;
                    vehicles[i].position -= direction * push_distance;
                    vehicles[i].speed = 0.0; // Stop the pushed vehicle
                    vehicles[i].target_speed = 0.0;
                    log_debug(&format!(
                        "[SEPARATION] Pushed V{} backward by {:.2} to separate from V{}",
                        v1_id, push_distance, v2_id
                    ));
                } else if v2_speed > v1_speed {
                    let push_distance = separation_force * delta_time;
                    vehicles[j].position += direction * push_distance;
                    vehicles[j].speed = 0.0; // Stop the pushed vehicle
                    vehicles[j].target_speed = 0.0;
                    log_debug(&format!(
                        "[SEPARATION] Pushed V{} backward by {:.2} to separate from V{}",
                        v2_id, push_distance, v1_id
                    ));
                } else {
                    // Both same speed (likely both stopped), push both apart
                    let push_distance = separation_force * delta_time * 0.5;
                    vehicles[i].position -= direction * push_distance;
                    vehicles[j].position += direction * push_distance;
                    log_debug(&format!(
                        "[SEPARATION] Pushed V{} and V{} apart by {:.2} each",
                        v1_id, v2_id, push_distance
                    ));
                }
            }
        }
    }
}

/// Initialize debug log (clear old data)
pub fn init_debug_log() {
    use std::fs::File;
    println!("[DEBUG] Initializing collision debug log...");
    match File::create("logs/collision_debug.log") {
        Ok(_) => {
            log_debug("=== COLLISION DEBUG LOG STARTED ===");
            println!("[DEBUG] Log file created successfully at logs/collision_debug.log");
        }
        Err(e) => {
            eprintln!("[ERROR] Failed to create debug log: {}", e);
        }
    }
}

/// Main vehicle update function
pub fn update_vehicles(
    vehicles: &mut [Vehicle],
    conflict_calendar: &mut ConflictCalendar,
    road_config: &RoadConfig,
    delta_time: f32,
    current_time: f32,
    stats_manager: &mut StatsManager,
) {
    // Log frame marker every 60 frames (once per second at 60fps)
    use std::sync::atomic::{AtomicU32, Ordering};
    static FRAME_COUNTER: AtomicU32 = AtomicU32::new(0);
    let frame = FRAME_COUNTER.fetch_add(1, Ordering::Relaxed);
    if frame % 60 == 0 {
        log_debug(&format!("\n--- FRAME {} (time={:.1}s) ---", frame, current_time));
    }
    
    let road_half_width: f32 = road_config.lane_width * road_config.num_lanes as f32;

    // Sensor zone parameters (adjustable for tuning)
    const SENSOR_LENGTH: f32 = 200.0;  // How far ahead to look
    const SENSOR_WIDTH: f32 = 80.0;    // Width of detection zone (increased to catch adjacent lanes)
    const EMERGENCY_DISTANCE: f32 = 80.0;  // Stop completely (increased from 60 to account for vehicle dimensions)
    const CRITICAL_DISTANCE: f32 = 120.0;  // Very slow (increased from 100)
    const WARNING_DISTANCE: f32 = 170.0;   // Slow down (increased from 150)
    
    // Track which vehicles are in emergency stop (to prevent override by second pass)
    let mut emergency_stopped: Vec<bool> = vec![false; vehicles.len()];
    // Track vehicles that were slowed/overridden by collision detection this frame
    let mut detection_active: Vec<bool> = vec![false; vehicles.len()];

    // First pass: 360-DEGREE collision detection for ALL vehicles (except right-turn)
    // This catches threats from ANY direction - critical for intersections!
    for i in 0..vehicles.len() {
        // Skip right-turn vehicles (they use dedicated lanes)
        let is_right_turn = vehicles[i].state == VehicleState::AlwaysMoving 
            && vehicles[i].route == crate::components::Route::Right;
        if is_right_turn {
            continue;
        }
        
        // Skip stopped/waiting vehicles
        if vehicles[i].state == VehicleState::Waiting || vehicles[i].state == VehicleState::Stopped {
            continue;
        }
        
        // Get vehicle info for the check (immutable borrow)
        let vehicle_id = vehicles[i].id;
        let vehicle_state = vehicles[i].state;
        let vehicle_direction = vehicles[i].direction;
        let vehicle_route = vehicles[i].route;
        let vehicle_position = vehicles[i].position;
        let vehicle_length = vehicles[i].length;
        let vehicle_speed = vehicles[i].speed;
        
        // CRITICAL: Much larger safety radius for EARLY detection
        // Old: 6.0 units - Too late! Vehicles already at 4.73 units before detection
        // New: 25.0 units - Detect threats much earlier, giving time to brake
        let safety_radius = 25.0;
        
        let mut closest_distance: Option<f32> = None;
        let mut closest_id: Option<u64> = None;
        
        for j in 0..vehicles.len() {
            if i == j {
                continue;
            }
            
            // Skip right-turn vehicles
            let other_is_right_turn = vehicles[j].state == VehicleState::AlwaysMoving 
                && vehicles[j].route == crate::components::Route::Right;
            if other_is_right_turn {
                continue;
            }
            
            let center_distance = (vehicle_position - vehicles[j].position).magnitude();
            let collision_threshold = (vehicle_length + vehicles[j].length) / 2.0; // ~4.0
            let safe_distance = collision_threshold + safety_radius; // ~29.0 units
            
            if center_distance < safe_distance {
                log_debug(&format!(
                    "[360_SCAN] V{} detected V{} at {:.2} units (threshold={:.2})",
                    vehicle_id, vehicles[j].id, center_distance, collision_threshold
                ));
                if closest_distance.is_none() || center_distance < closest_distance.unwrap() {
                    closest_distance = Some(center_distance);
                    closest_id = Some(vehicles[j].id);
                }
            }
        }
        
        // Now apply response (mutable access)
        if let (Some(other_id), Some(distance)) = (closest_id, closest_distance) {
            let collision_threshold = 4.0;
            
            // Calculate braking zones based on vehicle speed
            // Higher speed = need more distance to brake safely
            let speed_factor = vehicle_speed / 200.0; // Normalize to 0-1
            let critical_zone = collision_threshold + 6.0 * (1.0 + speed_factor); // 6-12 units (REDUCED from 8-16)
            let warning_zone = collision_threshold + 12.0 * (1.0 + speed_factor); // 12-24 units (REDUCED from 15-30)
            
            detection_active[i] = true;

            if distance < critical_zone {
                // CRITICAL: Emergency brake! Vehicle very close
                vehicles[i].speed = 0.0;
                vehicles[i].target_speed = 0.0;
                emergency_stopped[i] = true; // Mark as emergency stopped
                log_debug(&format!(
                    "[360_CRITICAL] V{}(state={:?},dir={:?},route={:?},speed={:.1}) EMERGENCY BRAKE - V{} at {:.2} units (critical_zone={:.2})",
                    vehicle_id, vehicle_state, vehicle_direction, vehicle_route, vehicle_speed, other_id, distance, critical_zone
                ));
            } else if distance < warning_zone {
                // WARNING: Slow down significantly but keep moving
                let safe_speed = SPEED_LEVELS[4].min(vehicle_speed); // Max 80 px/s (100.0)
                vehicles[i].speed = safe_speed;
                vehicles[i].target_speed = safe_speed;
                log_debug(&format!(
                    "[360_WARNING] V{} slowing to {:.1} px/s - V{} at {:.2} units",
                    vehicle_id, safe_speed, other_id, distance
                ));
            } else {
                // CAUTION: Reduce speed moderately
                let safe_speed = SPEED_LEVELS[3].min(vehicle_speed); // Max 150 px/s
                vehicles[i].target_speed = safe_speed;
                log_debug(&format!(
                    "[360_CAUTION] V{} reducing speed - V{} at {:.2} units",
                    vehicle_id, other_id, distance
                ));
            }
        }
    }
    
    // NEW: Universal collision avoidance using sensor zones
    // Works for ALL vehicle states and ALL directions
    // CRITICAL: This must run EVERY frame to prevent collisions
    // EXCEPT: Right-turn vehicles (AlwaysMoving + Right route) move at fixed speed and ignore obstacles
    for i in 0..vehicles.len() {
        let vehicle = &vehicles[i];
        
        // Skip stopped/waiting vehicles
        if vehicle.state == VehicleState::Waiting || vehicle.state == VehicleState::Stopped {
            continue;
        }
        
        // RIGHT-TURN OPTIMIZATION: Right-turn vehicles move at constant speed and ignore collision detection
        // They use dedicated right-turn lanes that never intersect with other traffic
        if vehicle.state == VehicleState::AlwaysMoving && vehicle.route == crate::components::Route::Right {
            // Set to constant medium speed and skip sensor detection
            vehicles[i].target_speed = SPEED_LEVELS[2]; // 200 px/s
            vehicles[i].speed = vehicles[i].speed.min(SPEED_LEVELS[2]);
            continue; // Skip collision detection entirely
        }

        let (detected, closest_distance) = vehicle.check_ahead(vehicles, SENSOR_LENGTH, SENSOR_WIDTH);
        
        if detected {
            if let Some(distance) = closest_distance {
                detection_active[i] = true;
                let action = if distance < EMERGENCY_DISTANCE {
                    "EMERGENCY_STOP"
                } else if distance < CRITICAL_DISTANCE {
                    "CRITICAL_SLOW"
                } else if distance < WARNING_DISTANCE {
                    "WARNING_SLOW"
                } else {
                    "CAUTION"
                };
                
                log_debug(&format!(
                    "[DETECTION] V{} (state={:?}, dir={:?}) detected obstacle at {:.1}px -> ACTION: {}",
                    vehicle.id, vehicle.state, vehicle.direction, distance, action
                ));
                log_debug(&format!(
                    "  BEFORE: speed={:.1}, target_speed={:.1}, position=({:.1}, {:.1})",
                    vehicle.speed, vehicle.target_speed, vehicle.position.x, vehicle.position.y
                ));
                
                // IMMEDIATE response - no gradual acceleration when obstacle detected
                if distance < EMERGENCY_DISTANCE {
                    // EMERGENCY: FULL STOP - lock both speed and target_speed
                    vehicles[i].speed = 0.0;
                    vehicles[i].target_speed = 0.0;
                    emergency_stopped[i] = true; // Mark as emergency stopped
                } else if distance < CRITICAL_DISTANCE {
                    // CRITICAL: Force very slow speed immediately
                    let safe_speed = SPEED_LEVELS[4];
                    vehicles[i].speed = vehicles[i].speed.min(safe_speed);
                    vehicles[i].target_speed = safe_speed;
                } else if distance < WARNING_DISTANCE {
                    // WARNING: Reduce to moderate speed
                    let safe_speed = SPEED_LEVELS[3];
                    if vehicles[i].speed > safe_speed {
                        vehicles[i].speed = safe_speed;
                    }
                    vehicles[i].target_speed = vehicles[i].target_speed.min(safe_speed);
                } else {
                    // CAUTION: Slight reduction
                    vehicles[i].target_speed = vehicles[i].target_speed.min(SPEED_LEVELS[2]);
                }
                
                log_debug(&format!(
                    "  AFTER: speed={:.1}, target_speed={:.1}",
                    vehicles[i].speed, vehicles[i].target_speed
                ));
            }
        }
    }
    
    // Conflict resolution based on conflict calendar (backup safety)

    let intersection_size = road_config.lane_width * road_config.num_lanes as f32;
    for v in vehicles.iter_mut() {
        if v.state != VehicleState::Moving {
            continue;
        }

        let mut min_capped_speed = v.target_speed;
        let mut conflict_found = false;
        for i in v.path_index..v.path.len() {
            let conflict_point = v.path[i];
            if conflict_point.x.abs() < intersection_size
                && conflict_point.y.abs() < intersection_size
            {
                let eta = VehiclePhysics::time_to_point(v, conflict_point);
                match conflict_calendar.try_reserve(
                    v.id,
                    conflict_point,
                    current_time + eta,
                    current_time,
                ) {
                    Ok(()) => {}
                    Err(gap) => {
                        let capped_speed = if gap.abs() < 0.5 {
                            SPEED_LEVELS[4]
                        } else if gap.abs() < 1.0 {
                            SPEED_LEVELS[3]
                        } else if gap.abs() < 2.0 {
                            SPEED_LEVELS[2]
                        } else {
                            SPEED_LEVELS[1]
                        };
                        min_capped_speed = min_capped_speed.min(capped_speed);
                        conflict_found = true;
                    }
                }
            }
        }
        if conflict_found {
            v.target_speed = min_capped_speed;
        }
    }

    // Second pass: normalize target speeds without undoing collision slowdowns
    // CRITICAL: Do NOT override emergency stops!
    // Runs after collision detection so detection_active can govern target-speed recovery
    for (idx, v) in vehicles.iter_mut().enumerate() {
        if v.state != VehicleState::Moving && v.state != VehicleState::AlwaysMoving {
            continue;
        }
        
        // CRITICAL: Skip vehicles in emergency stop
        if emergency_stopped[idx] {
            log_debug(&format!("[SECOND_PASS] Skipping V{} - in emergency stop", v.id));
            continue;
        }

        let base_free_speed = if v.state == VehicleState::AlwaysMoving {
            SPEED_LEVELS[2]
        } else {
            SPEED_LEVELS[1]
        };

        if !detection_active[idx] && v.target_speed < base_free_speed {
            v.target_speed = base_free_speed;
        }

        if let Some(waypoint) = v.get_current_waypoint() {
            let dist = (v.position - waypoint).magnitude();
            if dist < road_half_width * 0.5 {
                let approach_limit = speed_tier_from_distance(dist, road_half_width);
                if v.target_speed > approach_limit {
                    v.target_speed = approach_limit;
                }
            }
        }
    }

    // Apply physics
    for v in vehicles.iter_mut() {
        // Skip only truly stopped/waiting vehicles
        if v.state == VehicleState::Waiting || v.state == VehicleState::Stopped {
            continue;
        }

        let before_speed = v.speed;
        let before_target = v.target_speed;
        let before_pos = v.position;
        
        // Both Moving and AlwaysMoving vehicles use full physics
        // (AlwaysMoving can still slow down for obstacles via sensor detection)
        v.target_speed = quantize_with_hysteresis(v.speed, v.target_speed);
        VehiclePhysics::update_vehicle_physics(v, delta_time);
        
        // Log any vehicle that moves or changes speed
        if (v.position - before_pos).magnitude() > 0.1 || (v.speed - before_speed).abs() > 0.1 {
            log_debug(&format!(
                "[PHYSICS] V{} (state={:?}) target={:.1}->{:.1}, speed={:.1}->{:.1}, moved {:.1}px",
                v.id, v.state, before_target, v.target_speed, before_speed, v.speed,
                (v.position - before_pos).magnitude()
            ));
        }
    }
    
    // EMERGENCY SEPARATION: Push overlapping vehicles apart
    separate_overlapping_vehicles(vehicles, delta_time);

    check_vehicle_overlaps(vehicles, stats_manager);
}

/// Check if any vehicles have overlapping positions
/// EXCEPTION: Right-turn vehicles are excluded (they use dedicated lanes)
pub fn check_vehicle_overlaps(vehicles: &[Vehicle], stats_manager: &mut StatsManager) {
    use crate::components::Route;
    let mut overlaps_found = false;

    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let v1 = &vehicles[i];
            let v2 = &vehicles[j];
            
            // Skip if either vehicle is a right-turn vehicle
            let v1_is_right_turn = v1.state == VehicleState::AlwaysMoving && v1.route == Route::Right;
            let v2_is_right_turn = v2.state == VehicleState::AlwaysMoving && v2.route == Route::Right;
            
            if v1_is_right_turn || v2_is_right_turn {
                continue; // Right-turn vehicles use dedicated lanes, no overlap checking needed
            }

            let distance = (v1.position - v2.position).magnitude();

            let v1_half_length = v1.length / 2.0;
            let v2_half_length = v2.length / 2.0;
            let collision_threshold = v1_half_length + v2_half_length;
            let min_safe_distance = collision_threshold + 5.0;

            stats_manager.observe_distance(
                v1.id,
                v2.id,
                distance,
                collision_threshold,
                min_safe_distance,
            );

            // ONLY report ACTUAL collisions (overlapping hitboxes), not safety margin violations
            if distance < collision_threshold {
                if !overlaps_found {
                    println!("\n⚠️  === VEHICLE OVERLAP DETECTED ===");
                    log_debug("\n========== COLLISION OCCURRED ==========");
                    overlaps_found = true;
                }
                let msg = format!(
                    "[COLLISION] V{} and V{} overlap! Distance={:.2}, MinSafe={:.2}",
                    v1.id, v2.id, distance, min_safe_distance
                );
                println!(
                    "  Vehicle {} (state={:?}) at ({:.1}, {:.1}) and Vehicle {} (state={:?}) at ({:.1}, {:.1})",
                    v1.id, v1.state, v1.position.x, v1.position.y,
                    v2.id, v2.state, v2.position.x, v2.position.y
                );
                log_debug(&msg);
                log_debug(&format!(
                    "  V{}: state={:?}, dir={:?}, route={:?}, speed={:.1}, target={:.1}",
                    v1.id, v1.state, v1.direction, v1.route, v1.speed, v1.target_speed
                ));
                log_debug(&format!(
                    "  V{}: state={:?}, dir={:?}, route={:?}, speed={:.1}, target={:.1}",
                    v2.id, v2.state, v2.direction, v2.route, v2.speed, v2.target_speed
                ));
                println!(
                    "    Distance: {:.2} units (collision threshold: {:.2}) - ACTUAL COLLISION: {:.2} units overlap!",
                    distance,
                    collision_threshold,
                    collision_threshold - distance
                );
                println!(
                    "    V{} direction: {:?}, route: {:?}, speed: {:.1}",
                    v1.id, v1.direction, v1.route, v1.speed
                );
                println!(
                    "    V{} direction: {:?}, route: {:?}, speed: {:.1}",
                    v2.id, v2.direction, v2.route, v2.speed
                );
            }
        }
    }

    if overlaps_found {
        println!("=================================\n");
    }
}

/// Remove vehicles that have exited the simulation area
pub fn cleanup_vehicles(vehicles: &mut Vec<Vehicle>, cleanup_distance: f32) -> Vec<u64> {
    let mut removed = Vec::new();
    let mut index = 0;
    while index < vehicles.len() {
        let distance_from_center = vehicles[index].position.magnitude();
        if distance_from_center > cleanup_distance {
            removed.push(vehicles[index].id);
            vehicles.remove(index);
        } else {
            index += 1;
        }
    }
    removed
}


