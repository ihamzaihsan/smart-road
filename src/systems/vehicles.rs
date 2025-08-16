use bevy::prelude::*;
use crate::components::{Vehicle, VehicleState, Direction};
use crate::resources::{RoadConfig, SpawnManager, VehicleAssets, AutoSpawnTimer};
use rand::Rng;

/// System to handle keyboard input for manual vehicle spawning
pub fn handle_keyboard_spawning(
    keyboard_input: Res<Input<KeyCode>>,
    mut spawn_manager: ResMut<SpawnManager>,
    mut commands: Commands,
    road_config: Res<RoadConfig>,
    time: Res<Time>,
    existing_vehicles: Query<&Transform, With<Vehicle>>,
    vehicle_assets: Res<VehicleAssets>,
) {
    let current_time = time.elapsed_seconds_f64();

    // Check for arrow key presses
    let spawn_direction = if keyboard_input.just_pressed(KeyCode::Up) {
        Some(Direction::North)
    } else if keyboard_input.just_pressed(KeyCode::Down) {
        Some(Direction::South)
    } else if keyboard_input.just_pressed(KeyCode::Right) {
        Some(Direction::East)
    } else if keyboard_input.just_pressed(KeyCode::Left) {
        Some(Direction::West)
    } else {
        None
    };

    if let Some(direction) = spawn_direction {
        if spawn_manager.can_spawn(direction, current_time) {
            if let Some(spawn_pos) = find_safe_spawn_position_fixed_lane(
                direction,
                &spawn_manager,
                &road_config,
                &existing_vehicles,
            ) {
                spawn_vehicle_at_position(&mut commands, &mut spawn_manager, direction, spawn_pos, &vehicle_assets);
                spawn_manager.record_spawn(direction, current_time);
            }
        }
    }
}

/// System to handle automatic vehicle spawning at random intervals
pub fn handle_automatic_spawning(
    mut spawn_timer: ResMut<AutoSpawnTimer>,
    mut spawn_manager: ResMut<SpawnManager>,
    mut commands: Commands,
    road_config: Res<RoadConfig>,
    time: Res<Time>,
    existing_vehicles: Query<&Transform, With<Vehicle>>,
    vehicle_assets: Res<VehicleAssets>,
) {
    // Only proceed if automatic spawning is enabled
    if !spawn_timer.enabled {
        return;
    }

    // Update the spawn timer
    spawn_timer.timer.tick(time.delta());

    // Check if it's time to spawn a vehicle
    if spawn_timer.timer.finished() {
        let current_time = time.elapsed_seconds_f64();

        // Randomly select a direction to spawn from
        let direction = AutoSpawnTimer::random_direction();

        // Try to spawn a vehicle in the selected direction
        if spawn_manager.can_spawn(direction, current_time) {
            if let Some(spawn_pos) = find_safe_spawn_position(
                direction,
                &spawn_manager,
                &road_config,
                &existing_vehicles,
            ) {
                spawn_vehicle_at_position(&mut commands, &mut spawn_manager, direction, spawn_pos, &vehicle_assets);
                spawn_manager.record_spawn(direction, current_time);
            }
        }

        // Reset the timer with a fixed 1-second interval for the next spawn
        spawn_timer.reset_with_fixed_interval();
    }
}

/// Find a safe spawn position using fixed lane selection (for keyboard input)
/// Uses the first available lane in the direction (deterministic positioning)
fn find_safe_spawn_position_fixed_lane(
    direction: Direction,
    spawn_manager: &SpawnManager,
    road_config: &RoadConfig,
    existing_vehicles: &Query<&Transform, With<Vehicle>>,
) -> Option<Vec3> {
    let lane_centers = spawn_manager.get_lane_centers(direction, road_config);

    // Try lanes in order (fixed positioning for keyboard input)
    for &lane_offset in &lane_centers {
        let potential_pos = spawn_manager.get_spawn_position(direction, lane_offset);

        // Check if this position is safe (no vehicles too close)
        let is_safe = existing_vehicles.iter().all(|transform| {
            let distance = potential_pos.distance(transform.translation);
            distance >= spawn_manager.min_spawn_separation
        });

        if is_safe {
            return Some(potential_pos);
        }
    }

    None // No safe position found
}

/// Find a safe spawn position that doesn't collide with existing vehicles
/// Uses random lane selection - tries a random lane first, then falls back to other lanes if needed
fn find_safe_spawn_position(
    direction: Direction,
    spawn_manager: &SpawnManager,
    road_config: &RoadConfig,
    existing_vehicles: &Query<&Transform, With<Vehicle>>,
) -> Option<Vec3> {
    let lane_centers = spawn_manager.get_lane_centers(direction, road_config);

    // First try a random lane
    let random_lane_offset = spawn_manager.get_random_lane_center(direction, road_config);
    let potential_pos = spawn_manager.get_spawn_position(direction, random_lane_offset);

    // Check if this position is safe (no vehicles too close)
    let is_safe = existing_vehicles.iter().all(|transform| {
        let distance = potential_pos.distance(transform.translation);
        distance >= spawn_manager.min_spawn_separation
    });

    if is_safe {
        return Some(potential_pos);
    }

    // If random lane is not safe, try all other lanes as fallback
    for &lane_offset in &lane_centers {
        if (lane_offset - random_lane_offset).abs() < 0.1 {
            continue; // Skip the random lane we already tried
        }

        let potential_pos = spawn_manager.get_spawn_position(direction, lane_offset);

        let is_safe = existing_vehicles.iter().all(|transform| {
            let distance = potential_pos.distance(transform.translation);
            distance >= spawn_manager.min_spawn_separation
        });

        if is_safe {
            return Some(potential_pos);
        }
    }

    None // No safe position found
}

/// Spawn a vehicle at the specified position
fn spawn_vehicle_at_position(
    commands: &mut Commands,
    spawn_manager: &mut SpawnManager,
    direction: Direction,
    position: Vec3,
    vehicle_assets: &VehicleAssets,
) {
    let vehicle_id = spawn_manager.next_vehicle_id();
    let route = SpawnManager::random_route();
    let car_texture = vehicle_assets.get_random_texture();

    // Calculate rotation based on direction
    let rotation = match direction {
        Direction::North => Quat::from_rotation_z(0.0),           // Facing up
        Direction::South => Quat::from_rotation_z(std::f32::consts::PI), // Facing down
        Direction::East => Quat::from_rotation_z(-std::f32::consts::PI / 2.0), // Facing right
        Direction::West => Quat::from_rotation_z(std::f32::consts::PI / 2.0),  // Facing left
    };

    commands.spawn((
        SpriteBundle {
            texture: car_texture,
            sprite: Sprite {
                custom_size: Some(Vec2::new(60.0, 40.0)), // Increased vehicle size for better visibility
                ..default()
            },
            transform: Transform::from_translation(position).with_rotation(rotation),
            ..default()
        },
        Vehicle {
            id: vehicle_id,
            speed: 0.0,
            state: VehicleState::Moving,
            route,
            direction,
            target_speed: get_random_speed(),
        },
    ));
}



/// Get a random target speed for the vehicle
fn get_random_speed() -> f32 {
    let mut rng = rand::thread_rng();
    rng.gen_range(30.0..80.0) // Random speed between 30 and 80 units per second
}

/// System to move vehicles based on their direction and speed
pub fn move_vehicles(
    mut query: Query<(&mut Transform, &mut Vehicle)>,
    time: Res<Time>,
) {
    let dt = time.delta_seconds();

    for (mut transform, mut vehicle) in query.iter_mut() {
        if vehicle.state == VehicleState::Moving {
            // Gradually accelerate to target speed
            let speed_diff = vehicle.target_speed - vehicle.speed;
            let acceleration = 50.0; // units per second squared
            vehicle.speed += speed_diff.signum() * acceleration * dt;
            vehicle.speed = vehicle.speed.clamp(0.0, vehicle.target_speed);

            // Move the vehicle
            let movement = vehicle.direction.movement_vector() * vehicle.speed * dt;
            transform.translation += movement.extend(0.0);

            // Remove vehicles that have moved far from the intersection
            let distance_from_center = transform.translation.truncate().length();
            if distance_from_center > 500.0 {
                // Mark for cleanup (we'll implement cleanup later)
                vehicle.state = VehicleState::Stopped;
            }
        }
    }
}

/// Legacy function kept for compatibility with keyboard input
pub fn handle_spawn_input(
    _keyboard_input: Res<Input<KeyCode>>,
    _spawn_manager: ResMut<SpawnManager>,
    _commands: Commands,
    _road_config: Res<RoadConfig>,
    _time: Res<Time>,
    _existing_vehicles: Query<&Transform, With<Vehicle>>,
    _vehicle_assets: Res<VehicleAssets>,
) {
    // This function is now replaced by handle_automatic_spawning
    // Kept as stub for existing plugin compatibility
    // Does nothing - automatic spawning handles vehicle creation
}

/// Legacy function kept for compatibility
pub fn spawn_vehicles(_commands: Commands) {
    // This function is now replaced by handle_automatic_spawning
    // Kept as stub for existing plugin compatibility
}

pub fn load_vehicle_assets() {
    // stub: load textures / sprite sheets from assets/
}
