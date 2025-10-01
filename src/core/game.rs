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
    // Priority system tracking - now supports multiple simultaneous users
    current_intersection_users: Vec<(u64, Direction, Route)>, // ID, direction, and route of vehicles using intersection
    // Track last time each route was granted access to prevent same-route collisions
    last_route_access: std::collections::HashMap<(Direction, Route), Instant>,
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
            current_intersection_users: Vec::new(),
            last_route_access: std::collections::HashMap::new(),
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
    fn get_route_for_lane(&self, direction: Direction, lane_index: usize) -> Route {
        // Lanes are ordered from the left-most lane (index 0) to the right-most
        // for the direction returned by SpawnManager::get_lane_centers.
        // For North/West the lane indices map naturally: 0=Right,1=Straight,2=Left
        // For South/East the spawn manager flips lane ordering, so we need to
        // mirror the lane index to keep logical turning (right vs left) consistent.
        let max_lane_index = self.road_config.num_lanes.saturating_sub(1);
        let clamped = lane_index.min(max_lane_index);

        // Determine effective index depending on direction. For South and East
        // we mirror the index so that lane 0 becomes the right-most logically.
        let effective_index = match direction {
            Direction::South | Direction::East => max_lane_index.saturating_sub(clamped),
            _ => clamped,
        };

        match effective_index {
            // Inverted mapping: lane 0 -> Left, lane 1 -> Straight, lane 2 -> Right
            0 => Route::Left,
            1 => Route::Straight,
            2 => Route::Right,
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
            
            // Check if lane is full before attempting to spawn
            if self.is_lane_full(direction, lane_index) {
                continue; // Skip this lane, it's full
            }
            
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
    
    /// Check if a lane is full by counting vehicles in queue
    fn is_lane_full(&self, direction: Direction, lane_index: usize) -> bool {
        let queue_spacing = 50.0; // Match the spacing used in queue management
        let max_queue_length = 500.0; // Maximum distance from intersection for queue
        let max_vehicles_in_lane = (max_queue_length / queue_spacing) as usize;
        
        // Count vehicles in this specific lane approaching the intersection
        let vehicles_in_lane = self.vehicles.iter().filter(|v| {
            v.direction == direction && 
            v.lane_index == lane_index &&
            v.state == VehicleState::Waiting
        }).count();
        
        vehicles_in_lane >= max_vehicles_in_lane
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
            lane_index,
            direction,
            target_speed: SPEED_LEVELS[0],
            rotation,
            texture_index,
            path,
            path_index: 1,
            queue_target: None,
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

                // Determine the entry lane centers and compute the lane "rank" from center
                // (0 = innermost lane, larger = further from center). Then pick the exit
                // lane with the same rank so inner lanes map to inner lanes, outer to outer.
                let entry_lane_centers = self
                    .spawn_manager
                    .get_lane_centers(direction, &self.road_config);
                let exit_lane_centers = self
                    .spawn_manager
                    .get_lane_centers(exit_direction, &self.road_config);

                // Build arrays of indices sorted by absolute distance from center (ascending)
                let mut entry_indices: Vec<usize> = (0..entry_lane_centers.len()).collect();
                entry_indices.sort_by(|&a, &b| {
                    entry_lane_centers[a]
                        .abs()
                        .partial_cmp(&entry_lane_centers[b].abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                });

                let mut exit_indices: Vec<usize> = (0..exit_lane_centers.len()).collect();
                exit_indices.sort_by(|&a, &b| {
                    exit_lane_centers[a]
                        .abs()
                        .partial_cmp(&exit_lane_centers[b].abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                });

                // Find entry_rank: position of clamped_lane in the sorted-by-abs list
                let entry_rank = entry_indices
                    .iter()
                    .position(|&idx| idx == clamped_lane)
                    .unwrap_or(0);

                // Clamp rank to available exit lanes
                let chosen_rank = entry_rank.min(exit_indices.len().saturating_sub(1));

                let best_exit_lane = exit_indices[chosen_rank];

                // Use the chosen exit lane center for constructing turn points
                let turn_coordinate = self.spawn_manager.get_lane_center_by_index(
                    exit_direction,
                    best_exit_lane,
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

                let exit_far = self.exit_far_point(exit_direction, best_exit_lane);
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
        
        // First pass: ensure all Moving vehicles have reasonable target speeds
        for v in self.vehicles.iter_mut() {
            if v.state != VehicleState::Moving {
                continue;
            }
            
            // Ensure Moving vehicles always have at least minimum target speed
            if v.target_speed < SPEED_LEVELS[3] {
                v.target_speed = SPEED_LEVELS[1]; // Reset to reasonable speed (160 units/sec)
            }
            
            if let Some(waypoint) = v.get_current_waypoint() {
                let dist = (v.position - waypoint).magnitude();
                let base_limit = speed_tier_from_distance(dist, road_half_width);
                // Only apply speed reduction when very close to waypoint to avoid slow movement
                if dist < road_half_width * 0.5 {
                    v.target_speed = base_limit;
                } else {
                    // Allow normal cruising speed when not near waypoint
                    v.target_speed = v.target_speed.max(SPEED_LEVELS[1]); // At least medium speed
                }
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
            
            // Enhanced rear-end collision avoidance
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
                    
                    // More aggressive collision avoidance with larger detection range
                    if distance < safe_distance * 4.0 {
                        let safety_factor = (distance / safe_distance).clamp(0.05, 1.0);
                        
                        // Aggressive speed matching based on distance
                        let safe_speed = if distance < safe_distance * 0.8 {
                            // Very close - emergency slow down
                            other_vehicle.speed * 0.4
                        } else if distance < safe_distance * 1.5 {
                            // Close - match speed closely
                            other_vehicle.speed * 0.7
                        } else {
                            // Moderate distance - gradual speed adjustment
                            other_vehicle.speed * safety_factor * 0.9
                        };
                        
                        current_vehicle.target_speed = current_vehicle
                            .target_speed
                            .min(safe_speed.max(SPEED_LEVELS[4])); // Allow very slow speeds (40 units/sec)
                    }
                }
            }
        }
        // Priority-based intersection control
        self.apply_priority_based_intersection_control(delta_time);

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
        // Safety check: identify vehicles that are stuck (Moving but very slow with no good reason)
        let mut stuck_vehicle_ids: Vec<u64> = Vec::new();
        for i in 0..self.vehicles.len() {
            let v = &self.vehicles[i];
            if v.state != VehicleState::Moving {
                continue;
            }
            if v.target_speed < SPEED_LEVELS[3] && v.speed < SPEED_LEVELS[3] {
                // Check if there's a real obstacle ahead
                let has_close_obstacle = self.vehicles.iter().enumerate().any(|(j, other)| {
                    if i == j || other.state != VehicleState::Moving {
                        return false;
                    }
                    let distance = (v.position - other.position).magnitude();
                    let to_other = other.position - v.position;
                    let direction_dot = v.direction.movement_vector().dot(&to_other.normalize());
                    distance < 20.0 && direction_dot > 0.7
                });
                
                if !has_close_obstacle {
                    stuck_vehicle_ids.push(v.id);
                }
            }
        }
        
        // Apply physics and unstick vehicles
        for v in &mut self.vehicles {
            if v.state != VehicleState::Moving {
                continue;
            }
            v.target_speed = quantize_with_hysteresis(v.speed, v.target_speed);
            
            // Unstick vehicles that were identified as stuck
            if stuck_vehicle_ids.contains(&v.id) {
                v.target_speed = SPEED_LEVELS[3]; // 80 units/sec
            }
            
            VehiclePhysics::update_vehicle_physics(v, delta_time);
        }
        
        // Check for overlapping vehicle positions
        self.check_vehicle_overlaps();
    }
    
    /// Check if any vehicles have overlapping positions (considering their physical size)
    fn check_vehicle_overlaps(&self) {
        let mut overlaps_found = false;
        
        for i in 0..self.vehicles.len() {
            for j in (i + 1)..self.vehicles.len() {
                let v1 = &self.vehicles[i];
                let v2 = &self.vehicles[j];
                
                // Calculate distance between vehicles
                let distance = (v1.position - v2.position).magnitude();
                
                // Calculate minimum safe distance (considering both vehicles' sizes)
                // Use half-length of each vehicle plus a small safety margin
                let v1_half_length = v1.length / 2.0;
                let v2_half_length = v2.length / 2.0;
                let min_safe_distance = v1_half_length + v2_half_length + 2.0; // 2.0 is safety margin
                
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
                        distance, min_safe_distance, min_safe_distance - distance
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

    /// Check if two routes conflict in the intersection
    /// Returns true if the vehicles would interfere with each other
    fn routes_conflict(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
        // Same direction AND same route = definitely conflicts (would be in same path)
        if dir1 == dir2 && route1 == route2 {
            return true;
        }
        
        // Same direction but different routes = no conflict (different lanes/paths)
        if dir1 == dir2 {
            return false;
        }
        
        // Right turns never conflict with anyone from different directions (they use outer edge)
        if route1 == Route::Right && route2 == Route::Right {
            return false;
        }
        
        // Opposite straight lanes don't conflict (North-South or East-West)
        if route1 == Route::Straight && route2 == Route::Straight {
            match (dir1, dir2) {
                (Direction::North, Direction::South) | (Direction::South, Direction::North) => return false,
                (Direction::East, Direction::West) | (Direction::West, Direction::East) => return false,
                _ => {}
            }
        }
        
        // Right turn with straight from same axis (one going straight, one turning)
        // E.g., North-Right doesn't conflict with South-Straight
        if (route1 == Route::Right && route2 == Route::Straight) ||
           (route1 == Route::Straight && route2 == Route::Right) {
            match (dir1, dir2) {
                (Direction::North, Direction::South) | (Direction::South, Direction::North) => return false,
                (Direction::East, Direction::West) | (Direction::West, Direction::East) => return false,
                _ => {}
            }
        }
        
        // All other combinations conflict (left turns, perpendicular traffic, etc.)
        true
    }

    /// Apply priority-based intersection control
    fn apply_priority_based_intersection_control(&mut self, delta_time: f32) {
        let intersection_center = Vector2::new(0.0, 0.0);
        let intersection_half = self.road_config.lane_width * self.road_config.num_lanes as f32;
        // Increase max_queue_distance to accommodate larger queue_spacing of 120
        let max_queue_distance = intersection_half * 3.5; // Increased to fit proper spacing + buffer
        let min_queue_distance = intersection_half * 0.6;
        let approach_distance = self
            .simulation_config
            .detection_distance
            .clamp(min_queue_distance, max_queue_distance);
        let intersection_radius = intersection_half * 0.55; // Core intersection area

        // Clear vehicles that have exited the core area from active users list
        self.current_intersection_users.retain(|(user_id, _, _)| {
            self.vehicles.iter().any(|v| {
                v.id == *user_id
                    && (v.position - intersection_center).magnitude() < intersection_radius
            })
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

        for i in 0..self.vehicles.len() {
            let vehicle = &self.vehicles[i];

            // Skip if already using intersection
            if self.current_intersection_users.iter().any(|(id, _, _)| *id == vehicle.id) {
                continue;
            }

            let approach_vector = vehicle.direction.movement_vector();
            let relative_to_center = intersection_center - vehicle.position;
            let distance_along = relative_to_center.dot(&approach_vector);
            let inside_core =
                (vehicle.position - intersection_center).magnitude() < intersection_radius;

            if inside_core {
                continue;
            }

            if distance_along >= 0.0 && distance_along <= approach_distance {
                queue_candidates.push(Candidate {
                    index: i,
                    distance_along,
                    priority: Game::calculate_vehicle_priority(vehicle),
                    direction: vehicle.direction,
                    lane_index: vehicle.lane_index,
                });
            } else if vehicle.state == VehicleState::Waiting {
                release_indices.push(i);
            }
        }

        // Release vehicles that were waiting but have moved away from the queue zone
        for idx in release_indices {
            if let Some(vehicle) = self.vehicles.get_mut(idx) {
                vehicle.state = VehicleState::Moving;
                vehicle.queue_target = None;
                vehicle.target_speed = vehicle.target_speed.max(SPEED_LEVELS[2]);
            }
        }

        queue_candidates.sort_by(|a, b| {
            b.priority.cmp(&a.priority).then(
                a.distance_along
                    .partial_cmp(&b.distance_along)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
        });

        let mut waiting_candidate_indices: Vec<usize> = Vec::new();
        
        // Pre-compute moving vehicle positions to avoid borrow issues
        let moving_vehicles: Vec<(u64, Direction, Route, Vector2<f32>)> = self.vehicles.iter()
            .filter(|v| v.state == VehicleState::Moving)
            .map(|v| (v.id, v.direction, v.route, v.position))
            .collect();

        for (order, candidate) in queue_candidates.iter().enumerate() {
            let vehicle = &mut self.vehicles[candidate.index];
            let vehicle_id = vehicle.id;
            let vehicle_direction = vehicle.direction;
            let vehicle_route = vehicle.route;
            let vehicle_position = vehicle.position;
            let vehicle_movement_vector = vehicle.direction.movement_vector();
            
            // Skip if already using intersection
            if self.current_intersection_users.iter().any(|(id, _, _)| *id == vehicle_id) {
                continue;
            }

            // Check if this vehicle conflicts with any currently active vehicles
            let has_conflict = self.current_intersection_users.iter().any(|(_, active_dir, active_route)| {
                Self::routes_conflict(vehicle_direction, vehicle_route, *active_dir, *active_route)
            });
            
            // Check if there's a same-route vehicle too close ahead (prevent same-route collisions)
            // This creates a "safe zone" - don't release next vehicle until previous one is far enough
            let min_safe_distance = 500.0; // Safe distance with reduced speed variation (5 levels: 200-40)
            let has_same_route_nearby = moving_vehicles.iter().any(|(other_id, other_dir, other_route, other_pos)| {
                if *other_id == vehicle_id {
                    return false;
                }
                // Check if same direction and route
                if *other_dir == vehicle_direction && *other_route == vehicle_route {
                    let distance = (vehicle_position - other_pos).magnitude();
                    if distance < min_safe_distance {
                        // Check if other vehicle is ahead in the direction of travel
                        let to_other = other_pos - vehicle_position;
                        let direction_dot = vehicle_movement_vector.dot(&to_other.normalize());
                        return direction_dot > 0.3; // Vehicle is ahead
                    }
                }
                false
            });

            // Grant access logic:
            // - If intersection empty: grant to first vehicle (highest priority)
            // - If intersection has users: grant to any non-conflicting vehicle
            // - Don't grant if same-route vehicle is within safe zone ahead
            let is_intersection_empty = self.current_intersection_users.is_empty();
            let is_highest_priority_waiting = order == 0;
            
            let should_grant_access = if is_intersection_empty {
                is_highest_priority_waiting && !has_same_route_nearby
            } else {
                !has_conflict && !has_same_route_nearby
            };

            if should_grant_access {
                vehicle.state = VehicleState::Moving;
                vehicle.queue_target = None;
                vehicle.target_speed = SPEED_LEVELS[0]; // Set to maximum speed (200 units/sec)
                vehicle.speed = SPEED_LEVELS[3]; // Start slower (80 units/sec) for better control
                self.current_intersection_users.push((vehicle.id, vehicle.direction, vehicle.route));
            } else {
                vehicle.state = VehicleState::Waiting;
                vehicle.target_speed = 0.0;
                vehicle.speed = 0.0; // Ensure waiting vehicles are stopped
                waiting_candidate_indices.push(order);
            }
        }

        // Arrange waiting vehicles into per-lane queues with consistent spacing
        use std::collections::HashMap;
        let mut lane_waiters: HashMap<(Direction, usize), Vec<(usize, f32)>> = HashMap::new();

        for &candidate_idx in &waiting_candidate_indices {
            let candidate = &queue_candidates[candidate_idx];
            lane_waiters
                .entry((candidate.direction, candidate.lane_index))
                .or_insert_with(Vec::new)
                .push((candidate.index, candidate.distance_along));
        }

        let head_gap = 30.0; // Distance from entry boundary - balanced for visibility and safety
        // Reduced spacing to fit more cars in queue
        let queue_spacing = 70.0;  // Reduced from 120 to fit more vehicles

        for ((direction, lane_index), mut entries) in lane_waiters {
            entries.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

            let lane_center = self.spawn_manager.get_lane_center_by_index(
                direction,
                lane_index,
                &self.road_config,
            );
            let entry_boundary = Self::boundary_before(direction, intersection_half);
            let approach_vector = direction.movement_vector();

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
            if let Some(vehicle) = self.vehicles.get_mut(idx) {
                vehicle.queue_target = Some(target);
                vehicle.rotation = direction.rotation_angle();
                vehicle.speed = 0.0; // Stop immediately when in queue
                // Don't teleport - let update_waiting_vehicle_positions handle smooth movement
            }
        }

        self.update_waiting_vehicle_positions(delta_time);
    }

    fn update_waiting_vehicle_positions(&mut self, delta_time: f32) {
        let approach_speed = SPEED_LEVELS[2]; // Moderate speed for queue movement (120 units/sec)
        
        // First pass: identify which vehicles need to move
        let mut movements: Vec<(usize, Vector2<f32>, f32)> = Vec::new();
        
        for (idx, vehicle) in self.vehicles.iter().enumerate() {
            if vehicle.state == VehicleState::Waiting {
                if let Some(target) = vehicle.queue_target {
                    let to_target = target - vehicle.position;
                    let distance = to_target.magnitude();
                    
                    if distance > 1.0 {
                        // Check if there's a vehicle ahead blocking movement
                        let mut can_move = true;
                        let direction_vec = to_target / distance;
                        
                        for other in &self.vehicles {
                            if other.id != vehicle.id {
                                let to_other = other.position - vehicle.position;
                                let distance_to_other = to_other.magnitude();
                                
                                // Check if other vehicle is ahead in our direction of movement
                                if distance_to_other < 50.0 { // Safety check range
                                    let dot = to_other.normalize().dot(&direction_vec);
                                    if dot > 0.8 { // Other vehicle is ahead
                                        let min_safe = (vehicle.length + other.length) / 2.0 + 10.0;
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
                        // Close enough, snap to position
                        movements.push((idx, to_target / distance, distance));
                    }
                }
            }
        }
        
        // Second pass: apply movements
        for (idx, direction_vec, step) in movements {
            if let Some(vehicle) = self.vehicles.get_mut(idx) {
                vehicle.position += direction_vec * step;
                vehicle.speed = 0.0;
                
                // Snap to target if very close
                if let Some(target) = vehicle.queue_target {
                    if (target - vehicle.position).magnitude() < 0.5 {
                        vehicle.position = target;
                    }
                }
            }
        }
        
        // Clear queue targets for non-waiting vehicles
        for vehicle in &mut self.vehicles {
            if vehicle.state != VehicleState::Waiting {
                vehicle.queue_target = None;
            }
        }
    }

    /// Calculate vehicle priority based on route type
    fn calculate_vehicle_priority(vehicle: &Vehicle) -> u32 {
        match vehicle.route {
            Route::Right => 3,    // Highest priority - minimal intersection time
            Route::Straight => 2, // Medium priority
            Route::Left => 1,     // Lowest priority - longest intersection time
        }
    }

    /// Check if a vehicle is near the intersection
    fn is_near_intersection(&self, position: &Vector2<f32>) -> bool {
        let intersection_size = self.road_config.lane_width * self.road_config.num_lanes as f32;
        let approach_distance = intersection_size * 1.5; // Detection zone around intersection

        // Check if vehicle is within approach distance of intersection center
        position.magnitude() < approach_distance
    }

    pub fn render(&mut self, renderer: &mut Renderer) -> Result<(), String> {
        renderer.clear();
        renderer.render_roads(&self.road_config)?;
        renderer.render_vehicles(&self.vehicles, &self.vehicle_assets)?;
        renderer.present();
        Ok(())
    }
}
