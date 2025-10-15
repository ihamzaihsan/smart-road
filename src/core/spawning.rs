use crate::components::{Direction, Route, Vehicle, VehicleState};
use crate::core::physics::SPEED_LEVELS;
use crate::resources::{RoadConfig, SimulationConfig, VehicleAssets};
use crate::systems::SpawnManager;
use nalgebra::Vector2;
use sdl2::keyboard::Keycode;
use std::time::Instant;

/// Handle keyboard input for manual vehicle spawning
pub fn handle_keyboard_spawning(
    input_system: &crate::input::InputSystem,
    spawn_manager: &mut SpawnManager,
    vehicles: &[Vehicle],
    road_config: &RoadConfig,
    last_lane_used: &mut std::collections::HashMap<Direction, usize>,
    current_time: Instant,
    start_time: Instant,
) -> Option<(Direction, usize, Vector2<f32>, Route)> {
    let current_time_f64 = current_time.duration_since(start_time).as_secs_f64();
    let spawn_direction = if input_system.just_pressed(Keycode::Up) {
        Some(Direction::North)
    } else if input_system.just_pressed(Keycode::Down) {
        Some(Direction::South)
    } else if input_system.just_pressed(Keycode::Right) {
        Some(Direction::East)
    } else if input_system.just_pressed(Keycode::Left) {
        Some(Direction::West)
    } else {
        None
    };

    if let Some(direction) = spawn_direction {
        if spawn_manager.can_spawn(direction, current_time_f64) {
            if let Some((spawn_pos, lane_index)) = find_safe_spawn_position_with_lane(
                spawn_manager,
                vehicles,
                road_config,
                last_lane_used,
                direction,
            ) {
                let route = get_route_for_lane(direction, lane_index, road_config);
                spawn_manager.record_spawn(direction, current_time_f64);
                spawn_manager.record_lane_spawn(direction, lane_index, current_time_f64);
                return Some((direction, lane_index, spawn_pos, route));
            }
        }
    }
    None
}

/// Toggle automatic spawning on/off
pub fn handle_spawn_toggle(
    input_system: &crate::input::InputSystem,
    auto_spawn_timer: &mut crate::systems::AutoSpawnTimer,
    last_auto_spawn_at: &mut Instant,
) {
    if input_system.just_pressed(Keycode::R) {
        auto_spawn_timer.enabled = !auto_spawn_timer.enabled;
        if auto_spawn_timer.enabled {
            println!("🚗 Automatic spawning ENABLED");
            auto_spawn_timer.reset_timer();
            *last_auto_spawn_at = Instant::now();
        } else {
            println!("⏸️  Automatic spawning DISABLED - use arrow keys for manual spawning");
        }
    }
}

/// Handle automatic vehicle spawning based on timer
pub fn handle_automatic_spawning(
    auto_spawn_timer: &mut crate::systems::AutoSpawnTimer,
    spawn_manager: &mut SpawnManager,
    vehicles: &[Vehicle],
    road_config: &RoadConfig,
    simulation_config: &SimulationConfig,
    last_lane_used: &mut std::collections::HashMap<Direction, usize>,
    current_time: Instant,
    start_time: Instant,
    last_auto_spawn_at: Instant,
    min_auto_spawn_gap: std::time::Duration,
) -> Option<(Direction, usize, Vector2<f32>, Route)> {
    if !auto_spawn_timer.should_spawn() {
        return None;
    }
    if current_time.duration_since(last_auto_spawn_at) < min_auto_spawn_gap {
        return None;
    }
    if vehicles.len() >= simulation_config.max_vehicles {
        return None;
    }
    let direction = crate::systems::AutoSpawnTimer::random_direction();
    if let Some((spawn_pos, lane_index)) = find_safe_spawn_position_with_lane(
        spawn_manager,
        vehicles,
        road_config,
        last_lane_used,
        direction,
    ) {
        let current_time_f64 = current_time.duration_since(start_time).as_secs_f64();
        let route = get_route_for_lane(direction, lane_index, road_config);
        spawn_manager.record_spawn(direction, current_time_f64);
        spawn_manager.record_lane_spawn(direction, lane_index, current_time_f64);
        auto_spawn_timer.reset_timer();
        return Some((direction, lane_index, spawn_pos, route));
    }
    None
}

/// Find a safe spawn position in an available lane
fn find_safe_spawn_position_with_lane(
    spawn_manager: &SpawnManager,
    vehicles: &[Vehicle],
    road_config: &RoadConfig,
    last_lane_used: &mut std::collections::HashMap<Direction, usize>,
    direction: Direction,
) -> Option<(Vector2<f32>, usize)> {
    let lane_centers = spawn_manager.get_lane_centers(direction, road_config);

    // Get the last lane used, defaulting to a value that will start at lane 0
    let last_lane = last_lane_used.get(&direction).copied().unwrap_or(lane_centers.len().saturating_sub(1));

    // Try all lanes in rotation, starting from the NEXT lane after the last one used
    for offset in 0..lane_centers.len() {
        let lane_index = (last_lane + 1 + offset) % lane_centers.len();

        if is_lane_full(vehicles, direction, lane_index) {
            continue;
        }

        let lane_offset = lane_centers[lane_index];
        let potential_pos = spawn_manager.get_spawn_position(direction, lane_offset);
        let is_safe = vehicles.iter().all(|vehicle| {
            let distance = (potential_pos - vehicle.position).magnitude();
            distance >= spawn_manager.min_spawn_separation
        });

        if is_safe {
            // ALWAYS update the last lane used, even if spawning rapidly
            // This ensures proper rotation through all lanes
            last_lane_used.insert(direction, lane_index);
            return Some((potential_pos, lane_index));
        }
    }

    None
}

/// Check if a lane is full by counting vehicles in queue
fn is_lane_full(vehicles: &[Vehicle], direction: Direction, lane_index: usize) -> bool {
    let queue_spacing = 50.0;
    let max_queue_length = 500.0;
    let max_vehicles_in_lane = (max_queue_length / queue_spacing) as usize;

    let vehicles_in_lane = vehicles
        .iter()
        .filter(|v| {
            v.direction == direction
                && v.lane_index == lane_index
                && v.state == VehicleState::Waiting
        })
        .count();

    vehicles_in_lane >= max_vehicles_in_lane
}

/// Get the route for a specific lane based on direction
pub fn get_route_for_lane(
    direction: Direction,
    lane_index: usize,
    road_config: &RoadConfig,
) -> Route {
    let max_lane_index = road_config.num_lanes.saturating_sub(1);
    let clamped = lane_index.min(max_lane_index);

    let effective_index = match direction {
        Direction::South | Direction::East => max_lane_index.saturating_sub(clamped),
        _ => clamped,
    };

    match effective_index {
        0 => Route::Left,
        1 => Route::Straight,
        2 => Route::Right,
        _ => Route::Straight,
    }
}

/// Create a new vehicle at the specified position
pub fn spawn_vehicle_at_position(
    direction: Direction,
    lane_index: usize,
    position: Vector2<f32>,
    route: Route,
    path: Vec<Vector2<f32>>,
    spawn_manager: &mut SpawnManager,
    vehicle_assets: &VehicleAssets,
) -> Vehicle {
    let vehicle_id = spawn_manager.next_id;
    spawn_manager.next_id += 1;
    let texture_index = vehicle_assets.get_random_texture_index();
    let rotation = direction.rotation_angle();

    let initial_state = VehicleState::AlwaysMoving;
    let initial_speed = SPEED_LEVELS[2] as f32;
    let initial_target_speed = SPEED_LEVELS[2] as f32;

    Vehicle {
        id: vehicle_id,
        position,
        width: 2.0,
        length: 4.0,
        speed: initial_speed,
        state: initial_state,
        route,
        lane_index,
        direction,
        target_speed: initial_target_speed,
        rotation,
        texture_index,
        path,
        path_index: 1,
        queue_target: None,
    }
}
