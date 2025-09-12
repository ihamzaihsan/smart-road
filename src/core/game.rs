use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::render::TextureCreator;
use sdl2::video::WindowContext;
use sdl2::image::LoadTexture;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use nalgebra::Vector2;
use crate::components::{Vehicle, Direction, VehicleState, Route};
use crate::resources::{RoadConfig, VehicleAssets, SimulationConfig};
use crate::systems::{SpawnManager, AutoSpawnTimer};
use crate::rendering::Renderer;
use crate::input::InputSystem;
use crate::core::physics::{
    VehiclePhysics, quantize_with_hysteresis, speed_tier_from_distance, eta_cap_from_gap,
    ConflictCalendar, SPEED_LEVELS,
};

pub struct Game<'a> {
    pub vehicles: Vec<Vehicle>,
    pub spawn_manager: SpawnManager,
    pub auto_spawn_timer: AutoSpawnTimer,
    pub road_config: RoadConfig,
    pub simulation_config: SimulationConfig,
    pub vehicle_assets: VehicleAssets<'a>,
    pub input_system: InputSystem,
    pub last_update: Instant,
    pub start_time: Instant,
    pub conflict_calendar: ConflictCalendar,
    last_auto_spawn_at: Instant,
    min_auto_spawn_gap: Duration,
    next_lane_indices: HashMap<Direction, usize>,
}
impl<'a> Game<'a> {
    pub fn new(texture_creator: &'a TextureCreator<WindowContext>) -> Result<Self, String> {
        let mut vehicle_assets = VehicleAssets::new();
        let texture_names = vehicle_assets.texture_names.clone();
        for texture_name in texture_names {
            let texture_path = format!("assets/{}", texture_name);
            let texture = texture_creator.load_texture(&texture_path)
                .map_err(|e| format!("Failed to load texture {}: {}", texture_path, e))?;
            vehicle_assets.car_textures.push(texture);
        }
        let now = Instant::now();
        Ok(Self {
            vehicles: Vec::new(),
            spawn_manager: SpawnManager::default(),
            auto_spawn_timer: AutoSpawnTimer::default(),
            road_config: RoadConfig::default(),
            simulation_config: SimulationConfig::default(),
            vehicle_assets,
            input_system: InputSystem::new(),
            last_update: now,
            start_time: now,
            conflict_calendar: ConflictCalendar::new(),
            last_auto_spawn_at: now,
            min_auto_spawn_gap: Duration::from_secs_f32(0.8),
            next_lane_indices: HashMap::new(),
        })
    }
    pub fn update(&mut self, events: &[Event]) -> bool {
        let current_time = Instant::now();
        let delta_time = current_time.duration_since(self.last_update).as_secs_f32();
        self.last_update = current_time;
        self.input_system.update(events);
        for event in events {
            if let Event::Quit { .. } = event {
                return false;
            }
        }
        self.handle_keyboard_spawning(current_time);
        self.handle_spawn_toggle();
        if self.auto_spawn_timer.enabled {
            self.handle_automatic_spawning(current_time);
        }
        self.update_vehicles(delta_time);
        self.cleanup_vehicles();
        true
    }
    fn handle_keyboard_spawning(&mut self, current_time: Instant) {
        let current_time_f64 = current_time.duration_since(self.start_time).as_secs_f64();
        let spawn_direction = if self.input_system.just_pressed(Keycode::Up) {
            Some(Direction::North)
        } else if self.input_system.just_pressed(Keycode::Down) {
            Some(Direction::South)
        } else if self.input_system.just_pressed(Keycode::Right) {
            Some(Direction::East)
        } else if self.input_system.just_pressed(Keycode::Left) {
            Some(Direction::West)
        } else {
            None
        };
        if let Some(direction) = spawn_direction {
            if self.spawn_manager.can_spawn(direction, current_time_f64) {
                if let Some((spawn_pos, lane_index)) = self.find_safe_spawn_position(direction, current_time_f64) {
                    let route = self.get_route_for_arrow_spawn(direction);
                    self.spawn_vehicle_at_position(direction, spawn_pos, route);
                    self.spawn_manager.record_spawn(direction, current_time_f64);
                    self.spawn_manager.record_lane_spawn(direction, lane_index, current_time_f64);
                    let next_lane_index = (lane_index + 1) % self.road_config.num_lanes;
                    self.next_lane_indices.insert(direction, next_lane_index);
                }
            }
        }
    }
    fn get_route_for_arrow_spawn(&self, _direction: Direction) -> Route {
        Route::Straight
    }
    fn handle_spawn_toggle(&mut self) {
        if self.input_system.just_pressed(Keycode::R) {
            self.auto_spawn_timer.enabled = !self.auto_spawn_timer.enabled;
            if self.auto_spawn_timer.enabled {
                println!("🚗 Automatic spawning ENABLED ");
                self.auto_spawn_timer.reset_timer();
                self.last_auto_spawn_at = Instant::now();
            } else {
                println!("⏸️  Automatic spawning DISABLED - use arrow keys for manual spawning");
            }
        }
    }
    fn handle_automatic_spawning(&mut self, current_time: Instant) {
        if !self.auto_spawn_timer.should_spawn() {
            return;
        }
        if current_time.duration_since(self.last_auto_spawn_at) < self.min_auto_spawn_gap {
            return;
        }
        if self.vehicles.len() >= self.simulation_config.max_vehicles {
            return;
        }
        let current_time_f64 = current_time.duration_since(self.start_time).as_secs_f64();
        let direction = AutoSpawnTimer::random_direction();
        let route = AutoSpawnTimer::random_route();
        if let Some((spawn_pos, lane_index)) = self.find_safe_spawn_position(direction, current_time_f64) {
            self.spawn_vehicle_at_position(direction, spawn_pos, route);
            self.auto_spawn_timer.reset_timer();
            self.last_auto_spawn_at = current_time;
            let next_lane_index = (lane_index + 1) % self.road_config.num_lanes;
            self.next_lane_indices.insert(direction, next_lane_index);
        }
    }
    fn find_safe_spawn_position(&mut self, direction: Direction, current_time: f64) -> Option<(Vector2<f32>, usize)> {
        let lane_centers = self.spawn_manager.get_lane_centers(direction, &self.road_config);
        let num_lanes = lane_centers.len();
        if num_lanes == 0 {
            return None;
        }

        let start_lane_index = *self.next_lane_indices.get(&direction).unwrap_or(&0);

        for i in 0..num_lanes {
            let lane_index = (start_lane_index + i) % num_lanes;
            
            if !self.spawn_manager.can_spawn_in_lane(direction, lane_index, current_time) {
                continue;
            }

            let lane_offset = lane_centers[lane_index];
            let potential_pos = self.spawn_manager.get_spawn_position(direction, lane_offset);

            let is_safe = self.vehicles.iter().all(|vehicle| {
                (potential_pos - vehicle.position).magnitude() >= self.spawn_manager.min_spawn_separation
            });

            if is_safe {
                return Some((potential_pos, lane_index));
            }
        }

        None
    }
    fn spawn_vehicle_at_position(&mut self, direction: Direction, position: Vector2<f32>, route: Route) {
        let vehicle_id = self.spawn_manager.next_id;
        self.spawn_manager.next_id += 1;
        let texture_index = self.vehicle_assets.get_random_texture_index();
        let rotation = direction.rotation_angle();
        let path = self.generate_waypoints(direction, route, position);
        let vehicle = Vehicle {
            id: vehicle_id,
            position,
            speed: 0.0,
            state: VehicleState::Moving,
            route,
            direction,
            target_speed: SPEED_LEVELS[0],
            rotation,
            texture_index,
            path,
            path_index: 0,
            width: 20.0,
            length: 40.0,
        };
        self.vehicles.push(vehicle);
    }
    fn generate_waypoints(&self, direction: Direction, route: Route, start_pos: Vector2<f32>) -> Vec<Vector2<f32>> {
        let mut waypoints = Vec::new();
        let intersection_size = self.road_config.lane_width * self.road_config.num_lanes as f32;
        let lane_width = self.road_config.lane_width;
        let num_lanes = self.road_config.num_lanes as f32;

        let (entry_point, exit_point) = match (direction, route) {
            (Direction::North, Route::Straight) => (Vector2::new(start_pos.x, -intersection_size), Vector2::new(start_pos.x, intersection_size)),
            (Direction::South, Route::Straight) => (Vector2::new(start_pos.x, intersection_size), Vector2::new(start_pos.x, -intersection_size)),
            (Direction::East, Route::Straight) => (Vector2::new(-intersection_size, start_pos.y), Vector2::new(intersection_size, start_pos.y)),
            (Direction::West, Route::Straight) => (Vector2::new(intersection_size, start_pos.y), Vector2::new(-intersection_size, start_pos.y)),

            (Direction::North, Route::Left) => {
                let entry = Vector2::new(start_pos.x, -intersection_size);
                let exit = Vector2::new(-intersection_size, lane_width * (num_lanes - 0.5));
                waypoints.push(entry);
                waypoints.push(Vector2::new(entry.x, exit.y));
                (entry, exit)
            }
            (Direction::North, Route::Right) => {
                let entry = Vector2::new(start_pos.x, -intersection_size);
                let exit = Vector2::new(intersection_size, -lane_width * (num_lanes - 0.5));
                waypoints.push(entry);
                waypoints.push(Vector2::new(entry.x, exit.y));
                (entry, exit)
            }
            (Direction::South, Route::Left) => {
                let entry = Vector2::new(start_pos.x, intersection_size);
                let exit = Vector2::new(intersection_size, -lane_width * (num_lanes - 0.5));
                waypoints.push(entry);
                waypoints.push(Vector2::new(entry.x, exit.y));
                (entry, exit)
            }
            (Direction::South, Route::Right) => {
                let entry = Vector2::new(start_pos.x, intersection_size);
                let exit = Vector2::new(-intersection_size, lane_width * (num_lanes - 0.5));
                waypoints.push(entry);
                waypoints.push(Vector2::new(entry.x, exit.y));
                (entry, exit)
            }
            (Direction::East, Route::Left) => {
                let entry = Vector2::new(-intersection_size, start_pos.y);
                let exit = Vector2::new(-lane_width * (num_lanes - 0.5), -intersection_size);
                waypoints.push(entry);
                waypoints.push(Vector2::new(exit.x, entry.y));
                (entry, exit)
            }
            (Direction::East, Route::Right) => {
                let entry = Vector2::new(-intersection_size, start_pos.y);
                let exit = Vector2::new(lane_width * (num_lanes - 0.5), intersection_size);
                waypoints.push(entry);
                waypoints.push(Vector2::new(exit.x, entry.y));
                (entry, exit)
            }
            (Direction::West, Route::Left) => {
                let entry = Vector2::new(intersection_size, start_pos.y);
                let exit = Vector2::new(lane_width * (num_lanes - 0.5), intersection_size);
                waypoints.push(entry);
                waypoints.push(Vector2::new(exit.x, entry.y));
                (entry, exit)
            }
            (Direction::West, Route::Right) => {
                let entry = Vector2::new(intersection_size, start_pos.y);
                let exit = Vector2::new(-lane_width * (num_lanes - 0.5), -intersection_size);
                waypoints.push(entry);
                waypoints.push(Vector2::new(exit.x, entry.y));
                (entry, exit)
            }
        };

        if waypoints.is_empty() {
            waypoints.push(entry_point);
        }
        waypoints.push(exit_point);

        let final_destination = match route {
            Route::Straight => exit_point + direction.movement_vector() * 300.0,
            Route::Left => exit_point + direction.turn_left().movement_vector() * 300.0,
            Route::Right => exit_point + direction.turn_right().movement_vector() * 300.0,
        };
        waypoints.push(final_destination);

        waypoints
    }
    fn update_vehicles(&mut self, delta_time: f32) {
        let current_time = self.last_update.duration_since(self.start_time).as_secs_f32();
        let road_half_width: f32 = self.road_config.lane_width * self.road_config.num_lanes as f32;
        for v in self.vehicles.iter_mut() {
            if v.state != VehicleState::Moving {
                continue;
            }
            if let Some(waypoint) = v.get_current_waypoint() {
                let dist = (v.position - waypoint).magnitude();
                let base_limit = speed_tier_from_distance(dist, road_half_width);
                v.target_speed = base_limit;
            }
        }
        for i in 0..self.vehicles.len() {
            let (left, right) = self.vehicles.split_at_mut(i + 1);
            let current_vehicle = &mut left[i];
            if current_vehicle.state != VehicleState::Moving {
                continue;
            }
            let (emergency_brake, safe_speed) =
                VehiclePhysics::detect_collision_risk(current_vehicle, right);
            if emergency_brake {
                current_vehicle.target_speed = 0.0;
                continue;
            }
            current_vehicle.target_speed = current_vehicle.target_speed.min(safe_speed);
            for other_vehicle in right.iter() {
                if other_vehicle.state != VehicleState::Moving {
                    continue;
                }
                let in_same_lane = Self::are_vehicles_in_same_lane_static(current_vehicle, other_vehicle, &self.road_config);
                let is_following = Self::is_vehicle_following_static(current_vehicle, other_vehicle);
                if in_same_lane && is_following {
                    let distance = (current_vehicle.position - other_vehicle.position).magnitude();
                    let safe_distance = VehiclePhysics::calculate_safety_distance(current_vehicle, current_vehicle.speed);
                    if distance < safe_distance * 2.5 {
                        let safety_factor = (distance / safe_distance).clamp(0.2, 1.0);
                        let safe_speed = other_vehicle.speed * safety_factor * 0.7;
                        current_vehicle.target_speed = current_vehicle.target_speed.min(safe_speed.max(SPEED_LEVELS[10]));
                    }
                }
            }
        }
        for i in 0..self.vehicles.len() {
            let (left, remainder) = self.vehicles.split_at_mut(i);
            let (v, right) = remainder.split_at_mut(1);
            let v = &mut v[0];

            if v.state != VehicleState::Moving {
                continue;
            }

            let mut proactive_factor = 1.5;
            let mut closest_vehicle_dist = f32::MAX;

            let all_others = left.iter().chain(right.iter());
            for other in all_others {
                if Self::are_vehicles_in_same_lane_static(v, other, &self.road_config) && Self::is_vehicle_following_static(v, other) {
                    let distance = (v.position - other.position).magnitude();
                    if distance < closest_vehicle_dist {
                        closest_vehicle_dist = distance;
                    }
                }
            }

            if closest_vehicle_dist < f32::MAX {
                let safe_distance = VehiclePhysics::calculate_safety_distance(v, v.speed);
                proactive_factor = (closest_vehicle_dist / (safe_distance * 2.0)).clamp(1.0, 1.5);
            }

            let intersection_size = self.road_config.lane_width * self.road_config.num_lanes as f32;
            let mut min_capped_speed = v.target_speed;
            let mut conflict_found = false;
            for i in v.path_index..v.path.len() {
                let conflict_point = v.path[i];
                if conflict_point.x.abs() < intersection_size && conflict_point.y.abs() < intersection_size {
                    let eta = VehiclePhysics::time_to_point(v, conflict_point);
                    if let Err(delta_t) = self.conflict_calendar.try_reserve(v.id, conflict_point, eta, current_time) {
                        let capped_speed = eta_cap_from_gap(delta_t);
                        min_capped_speed = min_capped_speed.min(capped_speed);
                        conflict_found = true;
                        break;
                    }
                }
            }
            if conflict_found {
                v.target_speed = min_capped_speed;
            }
            let final_target_speed = quantize_with_hysteresis(v.speed, v.target_speed);
            v.target_speed = final_target_speed;
            VehiclePhysics::update_vehicle_physics(v, delta_time, proactive_factor);
        }
    }
    fn cleanup_vehicles(&mut self) {
        let cleanup_distance = 350.0;
        self.vehicles.retain(|vehicle| {
            let distance_from_center = vehicle.position.magnitude();
            distance_from_center <= cleanup_distance
        });
    }
    fn are_vehicles_in_same_lane_static(vehicle1: &Vehicle, vehicle2: &Vehicle, road_config: &RoadConfig) -> bool {
        if vehicle1.direction != vehicle2.direction {
            return false;
        }
        let lane_tolerance = road_config.lane_width * 0.8;
        match vehicle1.direction {
            Direction::North | Direction::South => {
                (vehicle1.position.x - vehicle2.position.x).abs() < lane_tolerance
            }
            Direction::East | Direction::West => {
                (vehicle1.position.y - vehicle2.position.y).abs() < lane_tolerance
            }
        }
    }
    fn is_vehicle_following_static(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let to_other = vehicle2.position - vehicle1.position;
        let movement_vector = vehicle1.direction.movement_vector();
        to_other.dot(&movement_vector) > 0.0
    }
    pub fn render(&mut self, renderer: &mut Renderer) -> Result<(), String> {
        renderer.clear();
        renderer.render_roads(&self.road_config)?;
        renderer.render_vehicles(&self.vehicles, &self.vehicle_assets)?;
        renderer.present();
        Ok(())
    }
}

