use crate::components::{Vehicle, VehicleState};
use crate::core::physics::{
    quantize_with_hysteresis, speed_tier_from_distance, ConflictCalendar, VehiclePhysics,
    SPEED_LEVELS,
};
use crate::resources::{RoadConfig, StatsManager};
use nalgebra::Vector2;
use std::fs::OpenOptions;
use std::io::Write;

fn log_debug(message: &str) {
    if let Ok(mut file) = OpenOptions::new()
        .create(true)
        .append(true)
        .open("logs/collision_debug.log")
    {
        let _ = writeln!(file, "{}", message);
    }
}

fn separate_overlapping_vehicles(vehicles: &mut [Vehicle], delta_time: f32) {
    use crate::components::Route;
    
    let collision_threshold = 4.0;
    let separation_force = 100.0;
    
    for i in 0..vehicles.len() {
        let v1_is_right_turn = vehicles[i].state == VehicleState::AlwaysMoving 
            && vehicles[i].route == Route::Right;
        
        for j in (i + 1)..vehicles.len() {
            let v2_is_right_turn = vehicles[j].state == VehicleState::AlwaysMoving 
                && vehicles[j].route == Route::Right;
            
            if v1_is_right_turn || v2_is_right_turn {
                continue;
            }
            
            let distance = (vehicles[i].position - vehicles[j].position).magnitude();
            
            if distance < collision_threshold {
                let v1_id = vehicles[i].id;
                let v2_id = vehicles[j].id;
                let v1_speed = vehicles[i].speed;
                let v2_speed = vehicles[j].speed;
                
                let direction = if distance > 0.01 {
                    (vehicles[j].position - vehicles[i].position) / distance
                } else {
                    Vector2::new(1.0, 0.0)
                };
                
                if v1_speed > v2_speed {
                    let push_distance = separation_force * delta_time;
                    vehicles[i].position -= direction * push_distance;
                    vehicles[i].speed = 0.0;
                    vehicles[i].target_speed = 0.0;
                    log_debug(&format!(
                        "[SEPARATION] Pushed V{} backward by {:.2} to separate from V{}",
                        v1_id, push_distance, v2_id
                    ));
                } else if v2_speed > v1_speed {
                    let push_distance = separation_force * delta_time;
                    vehicles[j].position += direction * push_distance;
                    vehicles[j].speed = 0.0;
                    vehicles[j].target_speed = 0.0;
                    log_debug(&format!(
                        "[SEPARATION] Pushed V{} backward by {:.2} to separate from V{}",
                        v2_id, push_distance, v1_id
                    ));
                } else {
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

pub fn update_vehicles(
    vehicles: &mut [Vehicle],
    conflict_calendar: &mut ConflictCalendar,
    road_config: &RoadConfig,
    delta_time: f32,
    current_time: f32,
    stats_manager: &mut StatsManager,
) {
    use std::sync::atomic::{AtomicU32, Ordering};
    static FRAME_COUNTER: AtomicU32 = AtomicU32::new(0);
    let frame = FRAME_COUNTER.fetch_add(1, Ordering::Relaxed);
    if frame % 60 == 0 {
        log_debug(&format!("\n--- FRAME {} (time={:.1}s) ---", frame, current_time));
    }
    
    let road_half_width: f32 = road_config.lane_width * road_config.num_lanes as f32;

    const SENSOR_LENGTH: f32 = 200.0;
    const SENSOR_WIDTH: f32 = 80.0;
    const EMERGENCY_DISTANCE: f32 = 80.0;
    const CRITICAL_DISTANCE: f32 = 120.0;
    const WARNING_DISTANCE: f32 = 170.0;
    
    let mut emergency_stopped: Vec<bool> = vec![false; vehicles.len()];
    let mut detection_active: Vec<bool> = vec![false; vehicles.len()];

    for i in 0..vehicles.len() {
        let is_right_turn = vehicles[i].state == VehicleState::AlwaysMoving 
            && vehicles[i].route == crate::components::Route::Right;
        if is_right_turn {
            continue;
        }
        
        if vehicles[i].state == VehicleState::Waiting || vehicles[i].state == VehicleState::Stopped {
            continue;
        }
        
        let vehicle_id = vehicles[i].id;
        let vehicle_state = vehicles[i].state;
        let vehicle_direction = vehicles[i].direction;
        let vehicle_route = vehicles[i].route;
        let vehicle_position = vehicles[i].position;
        let vehicle_length = vehicles[i].length;
        let vehicle_speed = vehicles[i].speed;
        
        let safety_radius = 25.0;
        
        let mut closest_distance: Option<f32> = None;
        let mut closest_id: Option<u64> = None;
        
        for j in 0..vehicles.len() {
            if i == j {
                continue;
            }
            
            let other_is_right_turn = vehicles[j].state == VehicleState::AlwaysMoving 
                && vehicles[j].route == crate::components::Route::Right;
            if other_is_right_turn {
                continue;
            }
            
            let center_distance = (vehicle_position - vehicles[j].position).magnitude();
            let collision_threshold = (vehicle_length + vehicles[j].length) / 2.0;
            let safe_distance = collision_threshold + safety_radius;
            
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
        
        if let (Some(other_id), Some(distance)) = (closest_id, closest_distance) {
            let collision_threshold = 4.0;
            let speed_factor = vehicle_speed / 200.0;
            let critical_zone = collision_threshold + 6.0 * (1.0 + speed_factor);
            let warning_zone = collision_threshold + 12.0 * (1.0 + speed_factor);
            
            detection_active[i] = true;

            if distance < critical_zone {
                vehicles[i].speed = 0.0;
                vehicles[i].target_speed = 0.0;
                emergency_stopped[i] = true;
                log_debug(&format!(
                    "[360_CRITICAL] V{}(state={:?},dir={:?},route={:?},speed={:.1}) EMERGENCY BRAKE - V{} at {:.2} units (critical_zone={:.2})",
                    vehicle_id, vehicle_state, vehicle_direction, vehicle_route, vehicle_speed, other_id, distance, critical_zone
                ));
            } else if distance < warning_zone {
                let safe_speed = SPEED_LEVELS[4].min(vehicle_speed);
                vehicles[i].speed = safe_speed;
                vehicles[i].target_speed = safe_speed;
                log_debug(&format!(
                    "[360_WARNING] V{} slowing to {:.1} px/s - V{} at {:.2} units",
                    vehicle_id, safe_speed, other_id, distance
                ));
            } else {
                let safe_speed = SPEED_LEVELS[3].min(vehicle_speed);
                vehicles[i].target_speed = safe_speed;
                log_debug(&format!(
                    "[360_CAUTION] V{} reducing speed - V{} at {:.2} units",
                    vehicle_id, other_id, distance
                ));
            }
        }
    }
    
    let mut mutual_blocks: Vec<(usize, usize, f32, f32)> = Vec::new();
    
    for i in 0..vehicles.len() {
        let vehicle = &vehicles[i];
        
        if vehicle.state == VehicleState::Waiting || vehicle.state == VehicleState::Stopped {
            continue;
        }
        
        if vehicle.state == VehicleState::AlwaysMoving && vehicle.route == crate::components::Route::Right {
            vehicles[i].target_speed = SPEED_LEVELS[2];
            vehicles[i].speed = vehicles[i].speed.min(SPEED_LEVELS[2]);
            continue;
        }

        let (detected, closest_distance) = vehicle.check_ahead(vehicles, SENSOR_LENGTH, SENSOR_WIDTH);
        
        if detected {
            if let Some(distance) = closest_distance {
                detection_active[i] = true;
                
                if distance < EMERGENCY_DISTANCE {
                    let sensor = crate::components::vehicle::SensorZone::new(&vehicle, SENSOR_LENGTH, SENSOR_WIDTH);
                    for j in 0..vehicles.len() {
                        if i == j || vehicles[j].route == crate::components::Route::Right {
                            continue;
                        }
                        if sensor.intersects_vehicle(&vehicles[j]) {
                            let (other_detected, other_distance) = vehicles[j].check_ahead(vehicles, SENSOR_LENGTH, SENSOR_WIDTH);
                            if other_detected && other_distance.is_some() && other_distance.unwrap() < EMERGENCY_DISTANCE {
                                let other_sensor = crate::components::vehicle::SensorZone::new(&vehicles[j], SENSOR_LENGTH, SENSOR_WIDTH);
                                if other_sensor.intersects_vehicle(&vehicle) {
                                    mutual_blocks.push((i, j, distance, other_distance.unwrap()));
                                    break;
                                }
                            }
                        }
                    }
                }
                
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
                
                if distance < EMERGENCY_DISTANCE {
                    vehicles[i].speed = 0.0;
                    vehicles[i].target_speed = 0.0;
                    emergency_stopped[i] = true;
                } else if distance < CRITICAL_DISTANCE {
                    let safe_speed = SPEED_LEVELS[4];
                    vehicles[i].speed = vehicles[i].speed.min(safe_speed);
                    vehicles[i].target_speed = safe_speed;
                } else if distance < WARNING_DISTANCE {
                    let safe_speed = SPEED_LEVELS[3];
                    if vehicles[i].speed > safe_speed {
                        vehicles[i].speed = safe_speed;
                    }
                    vehicles[i].target_speed = vehicles[i].target_speed.min(safe_speed);
                } else {
                    vehicles[i].target_speed = vehicles[i].target_speed.min(SPEED_LEVELS[2]);
                }
                
                log_debug(&format!(
                    "  AFTER: speed={:.1}, target_speed={:.1}",
                    vehicles[i].speed, vehicles[i].target_speed
                ));
            }
        }
    }
    
    for (idx1, idx2, _dist1, _dist2) in mutual_blocks {
        let dir1_priority = match vehicles[idx1].direction {
            crate::components::Direction::North => 3,
            crate::components::Direction::East => 2,
            crate::components::Direction::South => 1,
            crate::components::Direction::West => 0,
        };
        let dir2_priority = match vehicles[idx2].direction {
            crate::components::Direction::North => 3,
            crate::components::Direction::East => 2,
            crate::components::Direction::South => 1,
            crate::components::Direction::West => 0,
        };
        
        let (priority_idx, yield_idx) = if dir1_priority > dir2_priority {
            (idx1, idx2)
        } else if dir2_priority > dir1_priority {
            (idx2, idx1)
        } else {
            if vehicles[idx1].id < vehicles[idx2].id {
                (idx1, idx2)
            } else {
                (idx2, idx1)
            }
        };
        
        vehicles[priority_idx].speed = SPEED_LEVELS[4];
        vehicles[priority_idx].target_speed = SPEED_LEVELS[4];
        emergency_stopped[priority_idx] = false;
        
        vehicles[yield_idx].speed = 0.0;
        vehicles[yield_idx].target_speed = 0.0;
        emergency_stopped[yield_idx] = true;
        
        log_debug(&format!(
            "[MUTUAL_BLOCK_RESOLVED] V{} (dir={:?}) proceeds at slow speed, V{} (dir={:?}) yields",
            vehicles[priority_idx].id, vehicles[priority_idx].direction,
            vehicles[yield_idx].id, vehicles[yield_idx].direction
        ));
    }

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

    for (idx, v) in vehicles.iter_mut().enumerate() {
        if v.state != VehicleState::Moving && v.state != VehicleState::AlwaysMoving {
            continue;
        }
        
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

    for v in vehicles.iter_mut() {
        if v.state == VehicleState::Waiting || v.state == VehicleState::Stopped {
            continue;
        }

        let before_speed = v.speed;
        let before_target = v.target_speed;
        let before_pos = v.position;
        
        v.target_speed = quantize_with_hysteresis(v.speed, v.target_speed);
        VehiclePhysics::update_vehicle_physics(v, delta_time);
        
        if (v.position - before_pos).magnitude() > 0.1 || (v.speed - before_speed).abs() > 0.1 {
            log_debug(&format!(
                "[PHYSICS] V{} (state={:?}) target={:.1}->{:.1}, speed={:.1}->{:.1}, moved {:.1}px",
                v.id, v.state, before_target, v.target_speed, before_speed, v.speed,
                (v.position - before_pos).magnitude()
            ));
        }
    }
    
    separate_overlapping_vehicles(vehicles, delta_time);

    check_vehicle_overlaps(vehicles, stats_manager);
}

pub fn check_vehicle_overlaps(vehicles: &[Vehicle], stats_manager: &mut StatsManager) {
    use crate::components::Route;
    let mut overlaps_found = false;

    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let v1 = &vehicles[i];
            let v2 = &vehicles[j];
            
            let v1_is_right_turn = v1.state == VehicleState::AlwaysMoving && v1.route == Route::Right;
            let v2_is_right_turn = v2.state == VehicleState::AlwaysMoving && v2.route == Route::Right;
            
            if v1_is_right_turn || v2_is_right_turn {
                continue;
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


