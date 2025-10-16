use crate::components::{Direction, Route, Vehicle, VehicleState};
use crate::core::physics::SPEED_LEVELS;
use crate::core::waypoints;
use crate::resources::{RoadConfig, StatsManager};
use crate::systems::SpawnManager;
use nalgebra::Vector2;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IntersectionPhase {
    NorthSouth,
    EastWest,
    Clearing,
}

#[derive(Debug)]
pub struct IntersectionPhaseManager {
    pub current_phase: IntersectionPhase,
    pub previous_active_phase: IntersectionPhase,
    pub phase_start_time: f32,
    pub phase_duration: f32,
    pub clearing_duration: f32,
    pub max_vehicles_in_intersection: usize,
}

impl Default for IntersectionPhaseManager {
    fn default() -> Self {
        Self {
            current_phase: IntersectionPhase::NorthSouth,
            previous_active_phase: IntersectionPhase::NorthSouth,
            phase_start_time: 0.0,
            phase_duration: 5.0,
            clearing_duration: 0.5,
            max_vehicles_in_intersection: 6,
        }
    }
}

impl IntersectionPhaseManager {
    pub fn can_direction_enter(&self, direction: Direction) -> bool {
        match self.current_phase {
            IntersectionPhase::NorthSouth => {
                matches!(direction, Direction::North | Direction::South)
            }
            IntersectionPhase::EastWest => {
                matches!(direction, Direction::East | Direction::West)
            }
            IntersectionPhase::Clearing => false,
        }
    }

    pub fn update(&mut self, current_time: f32, vehicles_in_intersection: usize, waiting_counts: &HashMap<IntersectionPhase, usize>) {
        let time_in_phase = current_time - self.phase_start_time;

        match self.current_phase {
            IntersectionPhase::Clearing => {
                if vehicles_in_intersection == 0 || time_in_phase >= self.clearing_duration {
                    self.switch_to_next_active_phase(current_time);
                }
            }
            IntersectionPhase::NorthSouth | IntersectionPhase::EastWest => {
                let current_waiting = waiting_counts.get(&self.current_phase).copied().unwrap_or(0);
                let other_phase = match self.current_phase {
                    IntersectionPhase::NorthSouth => IntersectionPhase::EastWest,
                    IntersectionPhase::EastWest => IntersectionPhase::NorthSouth,
                    _ => IntersectionPhase::NorthSouth,
                };
                let other_waiting = waiting_counts.get(&other_phase).copied().unwrap_or(0);
                
                let immediate_switch = 
                    vehicles_in_intersection == 0 && current_waiting == 0 && other_waiting > 0;
                
                let should_switch = 
                    immediate_switch ||
                    (time_in_phase >= 1.0 && vehicles_in_intersection == 0) ||
                    time_in_phase >= self.phase_duration ||
                    (time_in_phase >= 1.5 && current_waiting == 0 && other_waiting > 0);

                if should_switch {
                    self.previous_active_phase = self.current_phase;
                    
                    if vehicles_in_intersection == 0 {
                        self.switch_to_next_active_phase(current_time);
                    } else {
                        self.current_phase = IntersectionPhase::Clearing;
                        self.phase_start_time = current_time;
                    }
                }
            }
        }
    }

    fn switch_to_next_active_phase(&mut self, current_time: f32) {
        self.current_phase = match self.previous_active_phase {
            IntersectionPhase::NorthSouth => IntersectionPhase::EastWest,
            IntersectionPhase::EastWest => IntersectionPhase::NorthSouth,
            IntersectionPhase::Clearing => IntersectionPhase::NorthSouth,
        };
        
        self.previous_active_phase = self.current_phase;
        self.phase_start_time = current_time;
    }
    

}

#[derive(Clone, Copy)]
pub struct ActiveIntersectionUser {
    pub id: u64,
    pub direction: Direction,
    pub route: Route,
    pub granted_at: f32,
    pub has_entered_core: bool,
    pub last_position: Vector2<f32>,  // Track position for stuck detection
    pub stuck_time: f32,
}

fn detect_deadlock(
    vehicles: &[Vehicle],
    current_intersection_users: &mut Vec<ActiveIntersectionUser>,
    _current_time: f32,
    delta_time: f32,
) -> bool {
    const STUCK_THRESHOLD: f32 = 5.0;
    const DEADLOCK_TIME: f32 = 3.0;
    
    let mut max_stuck_time = 0.0_f32;
    let mut vehicles_stuck_count = 0;
    
    for user in current_intersection_users.iter_mut() {
        if let Some(vehicle) = vehicles.iter().find(|v| v.id == user.id) {
            let movement = (vehicle.position - user.last_position).magnitude();
            
            if movement < STUCK_THRESHOLD {
                user.stuck_time += delta_time;
                if user.stuck_time > 1.0 {
                    vehicles_stuck_count += 1;
                }
                max_stuck_time = max_stuck_time.max(user.stuck_time);
            } else {
                user.stuck_time = 0.0;
            }
            
            user.last_position = vehicle.position;
        }
    }
    
    vehicles_stuck_count >= 2 && max_stuck_time >= DEADLOCK_TIME
}

fn resolve_deadlock(
    vehicles: &mut [Vehicle],
    current_intersection_users: &mut Vec<ActiveIntersectionUser>,
    _current_time: f32,
) {
    println!("[DEADLOCK] Resolving intersection deadlock at time {:.2}", _current_time);
    
    current_intersection_users.sort_by(|a, b| {
        b.stuck_time.partial_cmp(&a.stuck_time)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.granted_at.partial_cmp(&b.granted_at).unwrap_or(std::cmp::Ordering::Equal))
    });
    
    if let Some(priority_user) = current_intersection_users.first() {
        for vehicle in vehicles.iter_mut() {
            if vehicle.id == priority_user.id {
                vehicle.state = VehicleState::AlwaysMoving;
                vehicle.target_speed = SPEED_LEVELS[1];
                vehicle.speed = vehicle.speed.max(SPEED_LEVELS[3]);
                println!("[DEADLOCK] Prioritizing vehicle {} to clear intersection", vehicle.id);
            } else if current_intersection_users.iter().any(|u| u.id == vehicle.id) {
                if vehicle.state == VehicleState::Moving {
                    vehicle.target_speed = 0.0;
                    vehicle.speed = vehicle.speed * 0.5;
                }
            }
        }
    }
}

pub fn apply_priority_based_intersection_control(
    vehicles: &mut [Vehicle],
    current_intersection_users: &mut Vec<ActiveIntersectionUser>,
    phase_manager: &mut IntersectionPhaseManager,
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
    let exit_release_radius = intersection_half * 2.0;
    let max_hold_time = 8.0;

    if detect_deadlock(vehicles, current_intersection_users, current_time, delta_time) {
        resolve_deadlock(vehicles, current_intersection_users, current_time);
    }
    
    for user in current_intersection_users.iter_mut() {
        if let Some(vehicle) = vehicles.iter().find(|v| v.id == user.id) {
            let distance_to_center = (vehicle.position - intersection_center).magnitude();
            if !user.has_entered_core && distance_to_center <= intersection_radius {
                user.has_entered_core = true;
            }
        }
    }

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

    for i in 0..vehicles.len() {
        let vehicle = &vehicles[i];

        if current_intersection_users
            .iter()
            .any(|user| user.id == vehicle.id)
        {
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

        if distance_along >= 0.0 && distance_along <= approach_distance {
            stats_manager.record_detection(vehicle.id, current_time);
        }

        if vehicle.route == Route::Right {
            continue;
        }

        if inside_core {
            continue;
        }

        if distance_along >= 0.0 {
            if distance_along <= approach_distance {
                queue_candidates.push(Candidate {
                    index: i,
                    distance_along,
                    priority: calculate_vehicle_priority(vehicle),
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

    queue_candidates.sort_by(|a, b| {
        let priority_cmp = b.priority.cmp(&a.priority);
        if priority_cmp != std::cmp::Ordering::Equal {
            return priority_cmp;
        }
        
        let distance_diff = (a.distance_along - b.distance_along).abs();
        if distance_diff > 10.0 {
            return a.distance_along
                .partial_cmp(&b.distance_along)
                .unwrap_or(std::cmp::Ordering::Equal);
        }
        
        a.index.cmp(&b.index)
    });

    let mut waiting_candidate_indices: Vec<usize> = Vec::new();

    let moving_vehicles: Vec<(u64, Direction, Route, Vector2<f32>)> = vehicles
        .iter()
        .filter(|v| v.state == VehicleState::Moving)
        .map(|v| (v.id, v.direction, v.route, v.position))
        .collect();

    let vehicles_in_intersection = current_intersection_users
        .iter()
        .filter(|user| user.route != Route::Right)
        .count();

    let mut waiting_counts: HashMap<IntersectionPhase, usize> = HashMap::new();
    
    waiting_counts.insert(IntersectionPhase::NorthSouth, 0);
    waiting_counts.insert(IntersectionPhase::EastWest, 0);
    
    for candidate in &queue_candidates {
        let phase = if matches!(candidate.direction, Direction::North | Direction::South) {
            IntersectionPhase::NorthSouth
        } else {
            IntersectionPhase::EastWest
        };
        *waiting_counts.entry(phase).or_insert(0) += 1;
    }

    phase_manager.update(current_time, vehicles_in_intersection, &waiting_counts);

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

        let direction_allowed = phase_manager.can_direction_enter(vehicle_direction);
        
        let intersection_has_capacity = vehicles_in_intersection < phase_manager.max_vehicles_in_intersection;

        let has_conflict = current_intersection_users.iter().any(|active| {
            routes_conflict(
                vehicle_direction,
                vehicle_route,
                active.direction,
                active.route,
            )
        });

        let min_safe_distance = 250.0;
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

        let in_transition = phase_manager.current_phase == IntersectionPhase::Clearing;
        
        let should_grant_access = if is_intersection_empty && direction_allowed && !in_transition {
            order < 4 && !has_same_route_nearby
        } else {
            !in_transition &&
            direction_allowed &&
            intersection_has_capacity &&
            !has_conflict && 
            !has_same_route_nearby && 
            vehicles_in_intersection < 5
        };

        if should_grant_access {
            if vehicle_route == Route::Right {
                vehicle.state = VehicleState::AlwaysMoving;
            } else {
                vehicle.state = VehicleState::Moving;
            }
            
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
                    last_position: vehicle.position,
                    stuck_time: 0.0,
                });
            }
        } else {
            vehicle.state = VehicleState::Waiting;
            vehicle.target_speed = 0.0;
            vehicle.speed = 0.0;
            waiting_candidate_indices.push(order);
        }
    }

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

    let intersection_center = Vector2::new(0.0, 0.0);
    let intersection_radius = intersection_half / 1.5;
    let min_right_turn_spacing = 150.0;

    for i in 0..vehicles.len() {
        let vehicle = &vehicles[i];
        
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
            let mut has_vehicle_too_close = false;
            let min_safe_release_distance = vehicle.length * 5.0;
            
            for j in 0..vehicles.len() {
                if i == j {
                    continue;
                }
                let other = &vehicles[j];
                
                let distance = (vehicle.position - other.position).magnitude();
                if distance < min_safe_release_distance {
                    has_vehicle_too_close = true;
                    break;
                }
                
                // ALSO check if there's a vehicle ahead in same direction at larger distance
                if other.direction == vehicle.direction 
                    && (other.state == VehicleState::AlwaysMoving || other.state == VehicleState::Moving) 
                {
                    let to_other = other.position - vehicle.position;
                    let movement_vector = vehicle.direction.movement_vector();
                    let dot = to_other.dot(&movement_vector);
                    
                    // If other vehicle is ahead
                    if dot > 0.0 && distance < min_right_turn_spacing {
                        has_vehicle_too_close = true;
                        break;
                    }
                }
            }
            
            // Only release if no vehicle is too close
            if !has_vehicle_too_close {
                let vehicle = &mut vehicles[i];
                vehicle.state = VehicleState::AlwaysMoving;
                vehicle.target_speed = SPEED_LEVELS[2];
                vehicle.speed = SPEED_LEVELS[2];
                vehicle.queue_target = None;
            }
        }
    }

    update_waiting_vehicle_positions(vehicles, delta_time);
}

/// Update positions of waiting vehicles in queue
pub fn update_waiting_vehicle_positions(vehicles: &mut [Vehicle], delta_time: f32) {
    // Reduced approach speed for smoother queue movement
    let approach_speed = SPEED_LEVELS[2]; // Use slower speed (was SPEED_LEVELS[3])

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
                                        60.0
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
                        let max_step = (approach_speed * delta_time).min(distance).min(30.0);
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

pub fn routes_conflict(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    if dir1 == dir2 && route1 == route2 {
        return true;
    }

    if dir1 == dir2 {
        return false;
    }

    if route1 == Route::Right && route2 == Route::Right {
        return false;
    }

    if route1 == Route::Straight && route2 == Route::Straight {
        match (dir1, dir2) {
            (Direction::North, Direction::South) | (Direction::South, Direction::North) => {
                return false
            }
            (Direction::East, Direction::West) | (Direction::West, Direction::East) => {
                return false
            }
            _ => return true
        }
    }

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
            _ => return true  // Perpendicular right+straight can conflict
        }
    }

    // RULE 6: Right turn from perpendicular directions = safe
    if route1 == Route::Right || route2 == Route::Right {
        // Right turns generally don't conflict with other routes
        // except when paths directly cross
        return false;
    }

    // RULE 7: Left turns - MORE CONSERVATIVE
    // Multiple left turns from different directions often cause deadlocks
    if route1 == Route::Left && route2 == Route::Left {
        // Only allow opposite left turns
        match (dir1, dir2) {
            (Direction::North, Direction::South) | (Direction::South, Direction::North) |
            (Direction::East, Direction::West) | (Direction::West, Direction::East) => {
                return false  // Opposite left turns can proceed
            }
            _ => return true  // Perpendicular left turns conflict
        }
    }

    // RULE 8: Left turn + straight = CONFLICT for perpendicular
    if (route1 == Route::Left && route2 == Route::Straight)
        || (route1 == Route::Straight && route2 == Route::Left)
    {
        // Only allow if opposite directions
        match (dir1, dir2) {
            (Direction::North, Direction::South) | (Direction::South, Direction::North) |
            (Direction::East, Direction::West) | (Direction::West, Direction::East) => {
                return false  // Opposite directions safe
            }
            _ => return true  // Perpendicular left+straight conflict
        }
    }

    // RULE 9: Left turn + right turn
    if (route1 == Route::Left && route2 == Route::Right)
        || (route1 == Route::Right && route2 == Route::Left)
    {
        // Generally safe, but check for perpendicular conflicts
        return false;
    }

    // DEFAULT: When in doubt, mark as conflict to prevent deadlock
    true
}

/// Calculate vehicle priority based on route type
pub fn calculate_vehicle_priority(vehicle: &Vehicle) -> u32 {
    match vehicle.route {
        Route::Right => 3,
        Route::Straight => 2,
        Route::Left => 1,
    }
}
