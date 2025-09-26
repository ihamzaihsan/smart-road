use crate::components::{Direction, Route, Vehicle, VehicleState};
use crate::core::physics::{
    eta_cap_from_gap, quantize_with_hysteresis, speed_tier_from_distance, ConflictCalendar,
    VehiclePhysics, SPEED_LEVELS,
};
use crate::input::InputSystem;
use crate::rendering::Renderer;
use crate::resources::{RoadConfig, SimulationConfig, VehicleAssets};
use crate::systems::{AutoSpawnTimer, SpawnManager};
use nalgebra::Vector2;
use sdl2::event::Event;
use sdl2::image::LoadTexture;
use sdl2::keyboard::Keycode;
use sdl2::render::TextureCreator;
use sdl2::video::WindowContext;
use std::time::{Duration, Instant};
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
    last_lane_used: std::collections::HashMap<Direction, usize>,
}
impl<'a> Game<'a> {
    pub fn new(texture_creator: &'a TextureCreator<WindowContext>) -> Result<Self, String> {
        let mut vehicle_assets = VehicleAssets::new();
        let texture_names = vehicle_assets.texture_names.clone();
        for texture_name in texture_names {
            let texture_path = format!("assets/{}", texture_name);
            let texture = texture_creator
                .load_texture(&texture_path)
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
            last_lane_used: std::collections::HashMap::new(),
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
                // Find a safe spawn position using same logic as automatic spawning
                if let Some((spawn_pos, lane_index)) =
                    self.find_safe_spawn_position_with_lane(direction)
                {
                    let route = self.get_route_for_lane(direction, lane_index);
                    self.spawn_vehicle_at_position(direction, lane_index, spawn_pos, route);
                    self.spawn_manager.record_spawn(direction, current_time_f64);
                    self.spawn_manager
                        .record_lane_spawn(direction, lane_index, current_time_f64);
                }
            }
        }
    }
    fn get_route_for_lane(&self, _direction: Direction, lane_index: usize) -> Route {
        match lane_index {
            0 => Route::Right,
            1 => Route::Straight,
            2 => Route::Left,
            _ => Route::Straight,
        }
    }
    fn handle_spawn_toggle(&mut self) {
        if self.input_system.just_pressed(Keycode::R) {
            self.auto_spawn_timer.enabled = !self.auto_spawn_timer.enabled;
            if self.auto_spawn_timer.enabled {
                println!("🚗 Automatic spawning ENABLED (+1.0s throttle)");
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
        let direction = AutoSpawnTimer::random_direction();
        if let Some((spawn_pos, lane_index)) = self.find_safe_spawn_position_with_lane(direction) {
            let current_time_f64 = current_time.duration_since(self.start_time).as_secs_f64();
            let route = self.get_route_for_lane(direction, lane_index);
            self.spawn_vehicle_at_position(direction, lane_index, spawn_pos, route);
            self.spawn_manager.record_spawn(direction, current_time_f64);
            self.spawn_manager
                .record_lane_spawn(direction, lane_index, current_time_f64);
            self.auto_spawn_timer.reset_timer();
            self.last_auto_spawn_at = current_time;
        }
    }
    fn find_safe_spawn_position_with_lane(
        &mut self,
        direction: Direction,
    ) -> Option<(Vector2<f32>, usize)> {
        let lane_centers = self
            .spawn_manager
            .get_lane_centers(direction, &self.road_config);

        // Get the last used lane for this direction, or start with lane 0
        let last_lane = self.last_lane_used.get(&direction).copied().unwrap_or(0);

        // Try lanes starting from the next lane after the last used one (round-robin)
        for offset in 0..lane_centers.len() {
            let lane_index = (last_lane + 1 + offset) % lane_centers.len();
            let lane_offset = lane_centers[lane_index];
            let potential_pos = self
                .spawn_manager
                .get_spawn_position(direction, lane_offset);
            let is_safe = self.vehicles.iter().all(|vehicle| {
                let distance = (potential_pos - vehicle.position).magnitude();
                distance >= self.spawn_manager.min_spawn_separation
            });

            if is_safe {
                // Update the last used lane for this direction
                self.last_lane_used.insert(direction, lane_index);
                return Some((potential_pos, lane_index));
            }
        }

        None
    }
    fn spawn_vehicle_at_position(
        &mut self,
        direction: Direction,
        lane_index: usize,
        position: Vector2<f32>,
        route: Route,
    ) {
        let vehicle_id = self.spawn_manager.next_id;
        self.spawn_manager.next_id += 1;
        let texture_index = self.vehicle_assets.get_random_texture_index();
        let rotation = direction.rotation_angle();
        let path = self.generate_waypoints(direction, lane_index, route, position);
        let vehicle = Vehicle {
            id: vehicle_id,
            position,
            width: 2.0,
            length: 4.0,
            speed: 0.0,
            state: VehicleState::Moving,
            route,
            direction,
            target_speed: SPEED_LEVELS[0],
            rotation,
            texture_index,
            path,
            path_index: 1,
        };
        self.vehicles.push(vehicle);
    }

    fn generate_waypoints(
        &self,
        direction: Direction,
        lane_index: usize,
        route: Route,
        start_pos: Vector2<f32>,
    ) -> Vec<Vector2<f32>> {
        let mut waypoints = Vec::new();
        waypoints.push(start_pos);

        let max_lane_index = self.road_config.num_lanes.saturating_sub(1);
        let clamped_lane = lane_index.min(max_lane_index);
        let intersection_half = self.road_config.lane_width * self.road_config.num_lanes as f32;

        let movement_axis_is_y = matches!(direction, Direction::North | Direction::South);

        match route {
            Route::Straight => {
                let entry_boundary = Self::boundary_before(direction, intersection_half);
                let mut point = Self::set_axis(start_pos, movement_axis_is_y, entry_boundary);
                Self::push_if_new(&mut waypoints, point);

                let exit_boundary = Self::boundary_after(direction, intersection_half);
                point = Self::set_axis(point, movement_axis_is_y, exit_boundary);
                Self::push_if_new(&mut waypoints, point);

                let exit_far = self.exit_far_point(direction, clamped_lane);
                Self::push_if_new(&mut waypoints, exit_far);
            }
            Route::Right | Route::Left => {
                let exit_direction = match route {
                    Route::Right => direction.turn_right(),
                    Route::Left => direction.turn_left(),
                    Route::Straight => direction,
                };

                let turn_coordinate = self.spawn_manager.get_lane_center_by_index(
                    exit_direction,
                    clamped_lane,
                    &self.road_config,
                );
                let turn_point = Self::set_axis(start_pos, movement_axis_is_y, turn_coordinate);
                Self::push_if_new(&mut waypoints, turn_point);

                let exit_movement_axis_is_y =
                    matches!(exit_direction, Direction::North | Direction::South);
                let exit_boundary = Self::boundary_after(exit_direction, intersection_half);
                let corner_point =
                    Self::set_axis(turn_point, exit_movement_axis_is_y, exit_boundary);
                Self::push_if_new(&mut waypoints, corner_point);

                let exit_far = self.exit_far_point(exit_direction, clamped_lane);
                Self::push_if_new(&mut waypoints, exit_far);
            }
        }

        waypoints
    }

    fn push_if_new(points: &mut Vec<Vector2<f32>>, point: Vector2<f32>) {
        if points
            .last()
            .map_or(true, |last| (*last - point).magnitude_squared() > 1.0)
        {
            points.push(point);
        }
    }

    fn set_axis(point: Vector2<f32>, use_y: bool, value: f32) -> Vector2<f32> {
        if use_y {
            Vector2::new(point.x, value)
        } else {
            Vector2::new(value, point.y)
        }
    }

    fn boundary_before(direction: Direction, half_size: f32) -> f32 {
        match direction {
            Direction::North | Direction::East => -half_size,
            Direction::South | Direction::West => half_size,
        }
    }

    fn boundary_after(direction: Direction, half_size: f32) -> f32 {
        match direction {
            Direction::North | Direction::East => half_size,
            Direction::South | Direction::West => -half_size,
        }
    }

    fn exit_far_point(&self, direction: Direction, lane_index: usize) -> Vector2<f32> {
        let max_lane_index = self.road_config.num_lanes.saturating_sub(1);
        let clamped_lane = lane_index.min(max_lane_index);
        let lane_center =
            self.spawn_manager
                .get_lane_center_by_index(direction, clamped_lane, &self.road_config);
        let distance = self.simulation_config.cleanup_distance;
        match direction {
            Direction::North => Vector2::new(lane_center, distance),
            Direction::South => Vector2::new(lane_center, -distance),
            Direction::East => Vector2::new(distance, lane_center),
            Direction::West => Vector2::new(-distance, lane_center),
        }
    }
    fn update_vehicles(&mut self, delta_time: f32) {
        let current_time = self
            .last_update
            .duration_since(self.start_time)
            .as_secs_f32();
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
                let in_same_lane = Self::are_vehicles_in_same_lane_static(
                    current_vehicle,
                    other_vehicle,
                    &self.road_config,
                );
                let is_following =
                    Self::is_vehicle_following_static(current_vehicle, other_vehicle);
                if in_same_lane && is_following {
                    let distance = (current_vehicle.position - other_vehicle.position).magnitude();
                    let safe_distance =
                        VehiclePhysics::calculate_safety_distance(current_vehicle.speed);
                    if distance < safe_distance * 2.5 {
                        let safety_factor = (distance / safe_distance).clamp(0.2, 1.0);
                        let safe_speed = other_vehicle.speed * safety_factor * 0.7;
                        current_vehicle.target_speed = current_vehicle
                            .target_speed
                            .min(safe_speed.max(SPEED_LEVELS[10]));
                    }
                }
            }
        }
        for v in &mut self.vehicles {
            if v.state != VehicleState::Moving {
                continue;
            }
            let intersection_size = self.road_config.lane_width * self.road_config.num_lanes as f32;
            let mut min_capped_speed = v.target_speed;
            let mut conflict_found = false;
            for i in v.path_index..v.path.len() {
                let conflict_point = v.path[i];
                if conflict_point.x.abs() < intersection_size
                    && conflict_point.y.abs() < intersection_size
                {
                    let eta = VehiclePhysics::time_to_point(v, conflict_point);
                    match self.conflict_calendar.try_reserve(
                        v.id,
                        conflict_point,
                        current_time + eta,
                        current_time,
                    ) {
                        Ok(()) => {}
                        Err(gap) => {
                            let capped_speed = eta_cap_from_gap(gap);
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
        for v in &mut self.vehicles {
            if v.state != VehicleState::Moving {
                continue;
            }
            v.target_speed = quantize_with_hysteresis(v.speed, v.target_speed);
            VehiclePhysics::update_vehicle_physics(v, delta_time);
        }
    }
    fn cleanup_vehicles(&mut self) {
        let cleanup_distance = 650.0; // Larger than spawn distances (580 horizontal, 380 vertical)
        self.vehicles.retain(|vehicle| {
            let distance_from_center = vehicle.position.magnitude();
            distance_from_center <= cleanup_distance
        });
    }
    fn are_vehicles_in_same_lane_static(
        vehicle1: &Vehicle,
        vehicle2: &Vehicle,
        road_config: &RoadConfig,
    ) -> bool {
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
