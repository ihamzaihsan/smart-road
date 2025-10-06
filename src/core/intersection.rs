use crate::components::{Direction, Route, Vehicle, VehicleState};
use crate::core::physics::SPEED_LEVELS;
use crate::core::waypoints;
use crate::resources::{RoadConfig, StatsManager};
use crate::systems::SpawnManager;
use nalgebra::Vector2;
use std::collections::HashMap;

#[derive(Clone, Copy)]
pub struct ActiveIntersectionUser {
    pub id: u64,
    pub direction: Direction,
    pub route: Route,
    pub granted_at: f32,
    pub has_entered_core: bool,
}

/// Apply priority-based intersection control
pub fn apply_priority_based_intersection_control(
    vehicles: &mut [Vehicle],
    current_intersection_users: &mut Vec<ActiveIntersectionUser>,
    road_config: &RoadConfig,
    spawn_manager: &SpawnManager,
    delta_time: f32,
    current_time: f32,
    stats_manager: &mut StatsManager,
) {
    let intersection_center = Vector2::new(0.0, 0.0);
    let intersection_half = road_config.lane_width * road_config.num_lanes as f32;
    let max_queue_distance = intersection_half * 3.5;
    let min_queue_distance = intersection_half * 0.6;

    let direction_approach_distances: HashMap<Direction, f32> = [
        Direction::North,
        Direction::South,
        Direction::East,
        Direction::West,
    ]
    .into_iter()
    .map(|dir| {
        let lane_centers = spawn_manager.get_lane_centers(dir, road_config);
        let lane_offset = lane_centers
            .iter()
            .copied()
            .reduce(|a, b| if a.abs() > b.abs() { a } else { b })
            .unwrap_or(0.0);
        let spawn_pos = spawn_manager.get_spawn_position(dir, lane_offset);
        let approach_vector = dir.movement_vector();
        let available = (intersection_center - spawn_pos)
            .dot(&approach_vector)
            .abs();
        let approach = available.clamp(min_queue_distance, max_queue_distance);
        (dir, approach)
    })
    .collect();
    let intersection_radius = intersection_half * 0.55;
    let exit_release_radius = intersection_half * 1.5; // OPTIMIZED: Faster release (was 2.0)
    let max_hold_time = 4.0; // OPTIMIZED: Reduced timeout for faster turnover (was 8.0)

    // Update intersection core entry status
    for user in current_intersection_users.iter_mut() {
        if let Some(vehicle) = vehicles.iter().find(|v| v.id == user.id) {
            let distance_to_center = (vehicle.position - intersection_center).magnitude();
            if !user.has_entered_core && distance_to_center <= intersection_radius {
                user.has_entered_core = true;
            }
        }
    }

    // Remove vehicles that have cleared the intersection
    current_intersection_users.retain(|user| {
        if let Some(vehicle) = vehicles.iter().find(|v| v.id == user.id) {
            let distance_to_center = (vehicle.position - intersection_center).magnitude();
            let cleared_core = user.has_entered_core && distance_to_center > exit_release_radius;
            let timed_out = current_time - user.granted_at > max_hold_time;
            !(cleared_core || timed_out)
        } else {
            false
        }
    });

    struct Candidate {
        index: usize,
        distance_along: f32,
        priority: u32,
        direction: Direction,
        lane_index: usize,
    }

    let mut queue_candidates: Vec<Candidate> = Vec::new();
    let mut release_indices: Vec<usize> = Vec::new();
    let mut assigned_targets: Vec<(usize, Vector2<f32>, Direction)> = Vec::new();

    // OPTIMIZATION: Pre-allocate with expected capacity to reduce reallocation
    queue_candidates.reserve(20); // Expect ~20 vehicles near intersection
    release_indices.reserve(10);
    assigned_targets.reserve(20);

    // Identify vehicles approaching intersection
    for i in 0..vehicles.len() {
        let vehicle = &vehicles[i];

        // OPTIMIZATION: Skip vehicles already in intersection (early exit)
        if current_intersection_users
            .iter()
            .any(|user| user.id == vehicle.id)
        {
            continue;
        }

        // OPTIMIZATION: Skip right-turn vehicles early (they bypass queue)
        if vehicle.route == Route::Right {
            continue;
        }

        let approach_vector = vehicle.direction.movement_vector();
        let relative_to_center = intersection_center - vehicle.position;
        let distance_along = relative_to_center.dot(&approach_vector);
        let inside_core =
            (vehicle.position - intersection_center).magnitude() < intersection_radius;
        let approach_distance = direction_approach_distances
            .get(&vehicle.direction)
            .copied()
            .unwrap_or(max_queue_distance);

        // OPTIMIZATION: Record detection for stats
        if distance_along >= 0.0 && distance_along <= approach_distance {
            stats_manager.record_detection(vehicle.id, current_time);
        }

        // OPTIMIZATION: Skip vehicles inside intersection core
        if inside_core {
            continue;
        }

        if distance_along >= 0.0 {
            if distance_along <= approach_distance {
                queue_candidates.push(Candidate {
                    index: i,
                    distance_along,
                    priority: calculate_vehicle_priority(vehicle, current_time),
                    direction: vehicle.direction,
                    lane_index: vehicle.lane_index,
                });
            } else if vehicle.state == VehicleState::Waiting {
                release_indices.push(i);
            }
        } else if vehicle.state == VehicleState::Waiting {
            release_indices.push(i);
        }
    }

    // Release vehicles that moved away from queue zone
    for idx in release_indices {
        if let Some(vehicle) = vehicles.get_mut(idx) {
            vehicle.state = VehicleState::AlwaysMoving;
            vehicle.queue_target = None;
            vehicle.target_speed = SPEED_LEVELS[2] as f32;
            vehicle.speed = SPEED_LEVELS[2] as f32;
        }
    }

    // Sort candidates by priority then distance
    queue_candidates.sort_by(|a, b| {
        b.priority.cmp(&a.priority).then(
            a.distance_along
                .partial_cmp(&b.distance_along)
                .unwrap_or(std::cmp::Ordering::Equal),
        )
    });

    let mut waiting_candidate_indices: Vec<usize> = Vec::new();

    let moving_vehicles: Vec<(u64, Direction, Route, Vector2<f32>)> = vehicles
        .iter()
        .filter(|v| v.state == VehicleState::Moving)
        .map(|v| (v.id, v.direction, v.route, v.position))
        .collect();

    // Process candidates for intersection access
    for (order, candidate) in queue_candidates.iter().enumerate() {
        let vehicle = &mut vehicles[candidate.index];
        let vehicle_id = vehicle.id;
        let vehicle_direction = vehicle.direction;
        let vehicle_route = vehicle.route;
        let vehicle_position = vehicle.position;
        let vehicle_movement_vector = vehicle.direction.movement_vector();

        if current_intersection_users
            .iter()
            .any(|user| user.id == vehicle_id)
        {
            continue;
        }

        // OPTIMIZATION: Allow multiple non-conflicting vehicles simultaneously
        let has_conflict = current_intersection_users.iter().any(|active| {
            routes_conflict(
                vehicle_direction,
                vehicle_route,
                active.direction,
                active.route,
            )
        });

        let min_safe_distance = 300.0; 
        let has_same_route_nearby =
            moving_vehicles
                .iter()
                .any(|(other_id, other_dir, other_route, other_pos)| {
                    if *other_id == vehicle_id {
                        return false;
                    }
                    if *other_dir == vehicle_direction && *other_route == vehicle_route {
                        let distance = (vehicle_position - other_pos).magnitude();
                        if distance < min_safe_distance {
                            let to_other = other_pos - vehicle_position;
                            let direction_dot = vehicle_movement_vector.dot(&to_other.normalize());
                            return direction_dot > 0.3;
                        }
                    }
                    false
                });

        let is_intersection_empty = current_intersection_users.is_empty();
        let is_highest_priority_waiting = order == 0;

        // OPTIMIZATION: Allow multiple vehicles when intersection is empty
        // Grant access to top 3 highest priority vehicles if no conflicts
        let should_grant_access = if is_intersection_empty {
            // When empty, allow up to 3 vehicles with different non-conflicting routes
            (order < 3 || is_highest_priority_waiting) && !has_same_route_nearby
        } else {
            // When occupied, still allow non-conflicting vehicles
            !has_conflict && !has_same_route_nearby
        };

        if should_grant_access {
            vehicle.state = VehicleState::AlwaysMoving;
            vehicle.queue_target = None;
            vehicle.target_speed = SPEED_LEVELS[2] as f32;
            vehicle.speed = SPEED_LEVELS[2] as f32;

            if !current_intersection_users
                .iter()
                .any(|user| user.id == vehicle.id)
            {
                current_intersection_users.push(ActiveIntersectionUser {
                    id: vehicle.id,
                    direction: vehicle.direction,
                    route: vehicle.route,
                    granted_at: current_time,
                    has_entered_core: false,
                });
            }
        } else {
            vehicle.state = VehicleState::Waiting;
            vehicle.target_speed = 0.0;
            vehicle.speed = 0.0;
            waiting_candidate_indices.push(order);
        }
    }

    // Arrange waiting vehicles into per-lane queues
    let mut lane_waiters: HashMap<(Direction, usize), Vec<(usize, f32)>> = HashMap::new();

    for &candidate_idx in &waiting_candidate_indices {
        let candidate = &queue_candidates[candidate_idx];
        lane_waiters
            .entry((candidate.direction, candidate.lane_index))
            .or_insert_with(Vec::new)
            .push((candidate.index, candidate.distance_along));
    }

    let head_gap = 30.0; 
    let queue_spacing = 70.0; 

    for ((direction, lane_index), mut entries) in lane_waiters {
        entries.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        let lane_center =
            spawn_manager.get_lane_center_by_index(direction, lane_index, road_config);
        let entry_boundary = waypoints::boundary_before(direction, intersection_half);
        let approach_vector = direction.movement_vector();
        let approach_distance = direction_approach_distances
            .get(&direction)
            .copied()
            .unwrap_or(max_queue_distance);

        let front_position = if matches!(direction, Direction::North | Direction::South) {
            Vector2::new(lane_center, entry_boundary)
        } else {
            Vector2::new(entry_boundary, lane_center)
        } - approach_vector * head_gap;

        for (queue_pos, (vehicle_idx, _)) in entries.iter().enumerate() {
            let depth = head_gap + queue_spacing * queue_pos as f32;
            if depth > approach_distance {
                continue;
            }

            let target_position =
                front_position - approach_vector * (queue_spacing * queue_pos as f32);
            let target_distance = (intersection_center - target_position).dot(&approach_vector);
            if target_distance > approach_distance {
                continue;
            }

            assigned_targets.push((*vehicle_idx, target_position, direction));
        }
    }

    for (idx, target, direction) in assigned_targets {
        if let Some(vehicle) = vehicles.get_mut(idx) {
            vehicle.queue_target = Some(target);
            vehicle.rotation = direction.rotation_angle();
            vehicle.speed = 0.0;
        }
    }

    // Right-turn bypass: vehicles turning right don't need priority queue
    let intersection_center = Vector2::new(0.0, 0.0);
    let intersection_radius = intersection_half / 1.5;

    for vehicle in vehicles.iter_mut() {
        if vehicle.state != VehicleState::Waiting {
            continue;
        }

        if vehicle.route == Route::Right
            && !waypoints::has_intersection_point(
                &vehicle.path,
                intersection_center,
                intersection_radius,
            )
        {
            vehicle.state = VehicleState::AlwaysMoving;
            vehicle.target_speed = SPEED_LEVELS[2];
            vehicle.speed = SPEED_LEVELS[2];
            vehicle.queue_target = None;
        }
    }

    update_waiting_vehicle_positions(vehicles, delta_time);
}

/// Update positions of waiting vehicles in queue
pub fn update_waiting_vehicle_positions(vehicles: &mut [Vehicle], delta_time: f32) {
    let approach_speed = SPEED_LEVELS[3];

    let mut movements: Vec<(usize, Vector2<f32>, f32)> = Vec::new();

    for (idx, vehicle) in vehicles.iter().enumerate() {
        if vehicle.state == VehicleState::Waiting && vehicle.queue_target.is_some() {
            if let Some(target) = vehicle.queue_target {
                let to_target = target - vehicle.position;
                let distance = to_target.magnitude();

                if distance > 1.0 {
                    let mut can_move = true;
                    let direction_vec = to_target / distance;

                    for other in vehicles.iter() {
                        if other.id != vehicle.id {
                            let same_direction = vehicle.direction == other.direction;
                            if !same_direction {
                                continue;
                            }

                            let to_other = other.position - vehicle.position;
                            let distance_to_other = to_other.magnitude();

                            if distance_to_other < 100.0 {
                                let dot = to_other.normalize().dot(&direction_vec);
                                if dot > 0.7 {
                                    let min_safe = if other.state == VehicleState::Moving {
                                        150.0
                                    } else {
                                        (vehicle.length + other.length) / 2.0 + 15.0
                                    };

                                    if distance_to_other < min_safe {
                                        can_move = false;
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    if can_move {
                        let max_step = (approach_speed * delta_time).min(distance);
                        movements.push((idx, direction_vec, max_step));
                    }
                } else if distance > 0.1 {
                    movements.push((idx, to_target / distance, distance));
                }
            }
        }
    }

    for (idx, direction_vec, step) in movements {
        if let Some(vehicle) = vehicles.get_mut(idx) {
            if vehicle.state == VehicleState::Waiting {
                vehicle.position += direction_vec * step;
                vehicle.speed = 0.0;

                if let Some(target) = vehicle.queue_target {
                    if (target - vehicle.position).magnitude() < 0.5 {
                        vehicle.position = target;
                    }
                }
            }
        }
    }

    for vehicle in vehicles.iter_mut() {
        if vehicle.state != VehicleState::Waiting {
            vehicle.queue_target = None;
        }
    }
}

/// Check if two routes conflict in the intersection
/// OPTIMIZED: More permissive conflict detection to allow 3+ simultaneous vehicles
pub fn routes_conflict(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    // OPTIMIZATION: Same direction + same route = conflict (too close)
    if dir1 == dir2 && route1 == route2 {
        return true;
    }

    // OPTIMIZATION: Same direction, different routes = no conflict
    if dir1 == dir2 {
        return false;
    }

    // OPTIMIZATION: Both right turns = never conflict (separate arcs)
    if route1 == Route::Right && route2 == Route::Right {
        return false;
    }

    // OPTIMIZATION: Opposite straight paths = no conflict (parallel)
    if route1 == Route::Straight && route2 == Route::Straight {
        match (dir1, dir2) {
            (Direction::North, Direction::South) | (Direction::South, Direction::North) => {
                return false
            }
            (Direction::East, Direction::West) | (Direction::West, Direction::East) => {
                return false
            }
            _ => {}
        }
    }

    // OPTIMIZATION: Right turn + opposite straight = no conflict
    // Example: North-Right doesn't conflict with South-Straight
    if (route1 == Route::Right && route2 == Route::Straight)
        || (route1 == Route::Straight && route2 == Route::Right)
    {
        match (dir1, dir2) {
            (Direction::North, Direction::South) | (Direction::South, Direction::North) => {
                return false
            }
            (Direction::East, Direction::West) | (Direction::West, Direction::East) => {
                return false
            }
            _ => {}
        }
    }

    // OPTIMIZATION: Allow some Left+Straight combinations from different directions
    // North-Left + East-Straight = no conflict (different timing)
    if route1 == Route::Left && route2 == Route::Straight {
        match (dir1, dir2) {
            (Direction::North, Direction::East) | (Direction::East, Direction::North) => return false,
            (Direction::South, Direction::West) | (Direction::West, Direction::South) => return false,
            _ => {}
        }
    }
    if route1 == Route::Straight && route2 == Route::Left {
        match (dir1, dir2) {
            (Direction::North, Direction::West) | (Direction::West, Direction::North) => return false,
            (Direction::South, Direction::East) | (Direction::East, Direction::South) => return false,
            _ => {}
        }
    }

    // Default: assume conflict for safety
    true
}

/// Calculate vehicle priority based on route type with dynamic priority swapping
/// Every 1 seconds, swap priority between Straight and Left to prevent long waits
pub fn calculate_vehicle_priority(vehicle: &Vehicle, current_time: f32) -> u32 {
    // Swap priorities every 1 seconds
    let cycle_duration = 1.0;
    let cycle_phase = (current_time / cycle_duration) as u32 % 2;
    
    match vehicle.route {
        Route::Right => 3, // Always highest (they bypass anyway)
        Route::Straight => {
            if cycle_phase == 0 {
                2 // High priority (even cycles: 0-3s, 6-9s, etc.)
            } else {
                1 // Low priority (odd cycles: 3-6s, 9-12s, etc.)
            }
        }
        Route::Left => {
            if cycle_phase == 0 {
                1 // Low priority (even cycles)
            } else {
                2 // High priority (odd cycles)
            }
        }
    }
}
