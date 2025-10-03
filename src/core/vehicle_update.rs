use crate::components::{Vehicle, VehicleState};
use crate::core::physics::{
    quantize_with_hysteresis, speed_tier_from_distance, ConflictCalendar, VehiclePhysics,
    SPEED_LEVELS,
};
use crate::resources::{RoadConfig, StatsManager};

/// Main vehicle update function
pub fn update_vehicles(
    vehicles: &mut [Vehicle],
    conflict_calendar: &mut ConflictCalendar,
    road_config: &RoadConfig,
    delta_time: f32,
    current_time: f32,
    stats_manager: &mut StatsManager,
) {
    let road_half_width: f32 = road_config.lane_width * road_config.num_lanes as f32;

    // First pass: ensure all Moving/AlwaysMoving vehicles have reasonable target speeds
    for v in vehicles.iter_mut() {
        if v.state != VehicleState::Moving && v.state != VehicleState::AlwaysMoving {
            continue;
        }

        // AlwaysMoving vehicles maintain fixed speed - skip all modifications
        if v.state == VehicleState::AlwaysMoving {
            continue;
        }

        // Skip vehicles that just got permission and are still very slow
        if v.speed < SPEED_LEVELS[4] * 1.5 {
            continue;
        }

        // Ensure Moving vehicles always have at least minimum target speed
        if v.target_speed < SPEED_LEVELS[3] {
            v.target_speed = SPEED_LEVELS[1];
        }

        if let Some(waypoint) = v.get_current_waypoint() {
            let dist = (v.position - waypoint).magnitude();
            let base_limit = speed_tier_from_distance(dist, road_half_width);
            if dist < road_half_width * 0.5 {
                v.target_speed = base_limit;
            } else {
                v.target_speed = v.target_speed.max(SPEED_LEVELS[1]);
            }
        }
    }

    // Collision avoidance: check for vehicles ahead
    for i in 0..vehicles.len() {
        let vehicle = &vehicles[i];
        if vehicle.state != VehicleState::Moving && vehicle.state != VehicleState::AlwaysMoving {
            continue;
        }

        // AlwaysMoving vehicles (right-turn bypass) maintain FIXED speed
        if vehicle.state == VehicleState::AlwaysMoving {
            continue;
        }

        let mut target_speed_for_vehicles_ahead = vehicle.target_speed;

        for j in 0..vehicles.len() {
            if i == j {
                continue;
            }
            let other_vehicle = &vehicles[j];

            let in_same_lane =
                are_vehicles_in_same_lane_static(vehicle, other_vehicle, road_config);
            let is_following = is_vehicle_following_static(vehicle, other_vehicle);

            let distance = (vehicle.position - other_vehicle.position).magnitude();

            if in_same_lane && is_following {
                let max_detection = 800.0;

                if distance < max_detection {
                    let safe_distance = VehiclePhysics::calculate_safety_distance(vehicle.speed);
                    let critical_distance = safe_distance * 0.5;
                    let warning_distance = safe_distance * 1.0;
                    let caution_distance = safe_distance * 2.0;
                    let approach_distance = safe_distance * 4.0;

                    let base_target = if distance < critical_distance * 0.5 {
                        0.0
                    } else if distance < critical_distance {
                        SPEED_LEVELS[4]
                    } else if distance < warning_distance {
                        SPEED_LEVELS[3]
                    } else if distance < caution_distance {
                        SPEED_LEVELS[2]
                    } else if distance < approach_distance {
                        SPEED_LEVELS[1]
                    } else {
                        SPEED_LEVELS[0] * 0.9
                    };

                    let adjusted_target = if other_vehicle.state == VehicleState::Waiting
                        || other_vehicle.speed < 10.0
                    {
                        base_target as f32
                    } else {
                        (base_target as f32).min(other_vehicle.speed + 20.0)
                    };

                    target_speed_for_vehicles_ahead =
                        target_speed_for_vehicles_ahead.min(adjusted_target);
                    break;
                }
            }
        }

        vehicles[i].target_speed = target_speed_for_vehicles_ahead;

        // Emergency brake if very close to vehicle ahead
        if target_speed_for_vehicles_ahead < SPEED_LEVELS[3] {
            if vehicles[i].speed > target_speed_for_vehicles_ahead {
                vehicles[i].speed = target_speed_for_vehicles_ahead;
            }
        } else if target_speed_for_vehicles_ahead < SPEED_LEVELS[2] {
            if vehicles[i].speed > target_speed_for_vehicles_ahead * 1.1 {
                vehicles[i].speed = target_speed_for_vehicles_ahead * 1.1;
            }
        }
    }

    // Conflict resolution based on conflict calendar
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

    // Apply physics
    for v in vehicles.iter_mut() {
        if v.state == VehicleState::AlwaysMoving {
            VehiclePhysics::update_vehicle_position_only(v, delta_time);
            continue;
        }

        if v.state != VehicleState::Moving {
            continue;
        }
        v.target_speed = quantize_with_hysteresis(v.speed, v.target_speed);

        VehiclePhysics::update_vehicle_physics(v, delta_time);
    }

    check_vehicle_overlaps(vehicles, stats_manager);
}

/// Check if any vehicles have overlapping positions
pub fn check_vehicle_overlaps(vehicles: &[Vehicle], stats_manager: &mut StatsManager) {
    let mut overlaps_found = false;

    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let v1 = &vehicles[i];
            let v2 = &vehicles[j];

            let distance = (v1.position - v2.position).magnitude();

            let v1_half_length = v1.length / 2.0;
            let v2_half_length = v2.length / 2.0;
            let min_safe_distance = v1_half_length + v2_half_length + 5.0;
            let collision_threshold = v1_half_length + v2_half_length;

            stats_manager.observe_distance(
                v1.id,
                v2.id,
                distance,
                collision_threshold,
                min_safe_distance,
            );

            if distance < min_safe_distance {
                if !overlaps_found {
                    println!("\n⚠️  === VEHICLE OVERLAP DETECTED ===");
                    overlaps_found = true;
                }
                println!(
                    "  Vehicle {} (state={:?}) at ({:.1}, {:.1}) and Vehicle {} (state={:?}) at ({:.1}, {:.1})",
                    v1.id, v1.state, v1.position.x, v1.position.y,
                    v2.id, v2.state, v2.position.x, v2.position.y
                );
                println!(
                    "    Distance: {:.2} units (min safe: {:.2}) - OVERLAP: {:.2} units!",
                    distance,
                    min_safe_distance,
                    min_safe_distance - distance
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

/// Check if two vehicles are in the same lane
pub fn are_vehicles_in_same_lane_static(
    vehicle1: &Vehicle,
    vehicle2: &Vehicle,
    road_config: &RoadConfig,
) -> bool {
    if vehicle1.direction != vehicle2.direction {
        return false;
    }
    let lane_tolerance = road_config.lane_width * 0.70;
    let perpendicular_distance = match vehicle1.direction {
        crate::components::Direction::North | crate::components::Direction::South => {
            (vehicle1.position.x - vehicle2.position.x).abs()
        }
        crate::components::Direction::East | crate::components::Direction::West => {
            (vehicle1.position.y - vehicle2.position.y).abs()
        }
    };

    perpendicular_distance < lane_tolerance
}

/// Check if vehicle1 is following vehicle2
pub fn is_vehicle_following_static(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
    let to_other = vehicle2.position - vehicle1.position;
    let movement_vector = vehicle1.direction.movement_vector();
    let dot = to_other.dot(&movement_vector);
    dot > 0.0
}
