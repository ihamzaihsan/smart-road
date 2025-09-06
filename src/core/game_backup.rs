use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::render::TextureCreator;
use sdl2::video::WindowContext;
use sdl2::image::LoadTexture;
use std::time::{Duration, Instant};
use nalgebra::Vector2;

use crate::components::{Vehicle, Direction, VehicleState, Route};
use crate::resources::{RoadConfig, VehicleAssets};
use crate::systems::{SpawnManager, AutoSpawnTimer};
use crate::rendering::Renderer;
use crate::input::InputSystem;

use crate::core::physics::{
    VehiclePhysics, quantize_speed, quantize_with_hysteresis, speed_tier_from_distance, eta_cap_from_gap, 
    ConflictCalendar, SPEED_LEVELS,
};

pub struct Game<'a> {
    pub vehicles: Vec<Vehicle>,
    pub spawn_manager: SpawnManager,
    pub auto_spawn_timer: AutoSpawnTimer,
    pub road_config: RoadConfig,
    pub vehicle_assets: VehicleAssets<'a>,
    pub input_system: InputSystem,
    pub last_update: Instant,
    pub start_time: Instant,
    pub conflict_calendar: ConflictCalendar,

    // Throttle random auto-spawn by +1s (on top of timer's own cadence)
    last_auto_spawn_at: Instant,
    min_auto_spawn_gap: Duration,
}

impl<'a> Game<'a> {
    pub fn new(texture_creator: &'a TextureCreator<WindowContext>) -> Result<Self, String> {
        let mut vehicle_assets = VehicleAssets::new();

        // Load vehicle textures
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
            vehicle_assets,
            input_system: InputSystem::new(),
            last_update: now,
            start_time: now,
            conflict_calendar: ConflictCalendar::new(),
            last_auto_spawn_at: now,
            min_auto_spawn_gap: Duration::from_secs_f32(1.1), 
        })
    }

    pub fn update(&mut self, events: &[Event]) -> bool {
        let current_time = Instant::now();
        let delta_time = current_time.duration_since(self.last_update).as_secs_f32();
        self.last_update = current_time;

        // Update input system
        self.input_system.update(events);

        // Check for quit events
        for event in events {
            if let Event::Quit { .. } = event {
                return false; // Signal to quit
            }
        }

        // Handle keyboard spawning
        self.handle_keyboard_spawning(current_time);

        // Handle automatic spawning toggle
        self.handle_spawn_toggle();

        // Handle automatic spawning (throttled)
        if self.auto_spawn_timer.enabled {
            self.handle_automatic_spawning(current_time);
        }

        // Update vehicle movement with checkpoint & anti-collision policy
        self.update_vehicles(delta_time);

        // Clean up vehicles that have left the screen
        self.cleanup_vehicles();

        true // Continue running
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
                // Find an available lane with cooldown support
                if let Some(lane_index) = self.spawn_manager.find_available_lane(direction, &self.road_config, current_time_f64) {
                    let lane_offset = self.spawn_manager.get_lane_center_by_index(direction, lane_index, &self.road_config);
                    let spawn_pos = self.spawn_manager.get_spawn_position(direction, lane_offset);
                    
                    // Check spatial separation
                    let is_safe = self.vehicles.iter().all(|vehicle| {
                        let distance = (spawn_pos - vehicle.position).magnitude();
                        distance >= self.spawn_manager.min_spawn_separation
                    });
                    
                    if is_safe {
                        self.spawn_vehicle_at_position(direction, spawn_pos);
                        self.spawn_manager.record_spawn(direction, current_time_f64);
                        self.spawn_manager.record_lane_spawn(direction, lane_index, current_time_f64);
                    }
                }
            }
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

        if let Some(spawn_pos) = self.find_safe_spawn_position_random() {
            let direction = AutoSpawnTimer::random_direction();
            self.spawn_vehicle_at_position(direction, spawn_pos);
            self.auto_spawn_timer.reset_timer();
            self.last_auto_spawn_at = current_time;
        }
    }

    fn find_safe_spawn_position_random(&self) -> Option<Vector2<f32>> {
        let direction = AutoSpawnTimer::random_direction();
        if let Some(spawn_pos) = self.find_safe_spawn_position(direction) {
            return Some(spawn_pos);
        }
        None
    }

    fn find_safe_spawn_position(&self, direction: Direction) -> Option<Vector2<f32>> {
        let lane_centers = self.spawn_manager.get_lane_centers(direction, &self.road_config);

        let random_lane_offset = self.spawn_manager.get_random_lane_center(direction, &self.road_config);
        let potential_pos = self.spawn_manager.get_spawn_position(direction, random_lane_offset);

        let is_safe = self.vehicles.iter().all(|vehicle| {
            let distance = (potential_pos - vehicle.position).magnitude();
            distance >= self.spawn_manager.min_spawn_separation
        });
        
        if is_safe {
            return Some(potential_pos);
        }

        for &lane_offset in &lane_centers {
            if (lane_offset - random_lane_offset).abs() < 0.1 {
                continue; 
            }
            
            let potential_pos = self.spawn_manager.get_spawn_position(direction, lane_offset);
            
            let is_safe = self.vehicles.iter().all(|vehicle| {
                let distance = (potential_pos - vehicle.position).magnitude();
                distance >= self.spawn_manager.min_spawn_separation
            });
            
            if is_safe {
                return Some(potential_pos);
            }
        }
        
        None
    }

    fn spawn_vehicle_at_position(&mut self, direction: Direction, position: Vector2<f32>) {
        let vehicle_id = self.spawn_manager.next_id;
        self.spawn_manager.next_id += 1;

        let texture_index = self.vehicle_assets.get_random_texture_index();
        let rotation = direction.rotation_angle();

        // Generate waypoints for the vehicle based on its route
        let path = self.generate_waypoints(direction, Route::Straight, position);

        let vehicle = Vehicle {
            id: vehicle_id,
            position,
            speed: 0.0,
            state: VehicleState::Moving,
            route: Route::Straight,
            direction,
            // Start by aiming for the fastest tier; checkpoint logic will cap this each frame.
            target_speed: SPEED_LEVELS[0], // 600
            rotation,
            texture_index,
            path,
            path_index: 0,
        };

        self.vehicles.push(vehicle);
    }

    fn generate_waypoints(&self, direction: Direction, _route: Route, start_pos: Vector2<f32>) -> Vec<Vector2<f32>> {
        let mut waypoints = Vec::new();
        
        // For now, generate a simple straight path to the opposite side
        // This can be enhanced later to support turns
        match direction {
            Direction::North => {
                waypoints.push(Vector2::new(start_pos.x, 0.0)); // Center
                waypoints.push(Vector2::new(start_pos.x, 300.0)); // Exit
            }
            Direction::South => {
                waypoints.push(Vector2::new(start_pos.x, 0.0)); // Center
                waypoints.push(Vector2::new(start_pos.x, -300.0)); // Exit
            }
            Direction::East => {
                waypoints.push(Vector2::new(0.0, start_pos.y)); // Center
                waypoints.push(Vector2::new(300.0, start_pos.y)); // Exit
            }
            Direction::West => {
                waypoints.push(Vector2::new(0.0, start_pos.y)); // Center
                waypoints.push(Vector2::new(-300.0, start_pos.y)); // Exit
            }
        }
        
        waypoints
    }

    fn compute_checkpoints(&self) -> Vec<Vector2<f32>> {
        fn symmetric_offsets(mut vals: Vec<f32>, lane_width: f32) -> Vec<f32> {
            let mut out: Vec<f32> = vec![0.0];
            out.extend(vals.drain(..));

            let mut mirrors: Vec<f32> = out
                .iter()
                .copied()
                .filter(|v| v.abs() > 0.1)
                .map(|v| -v)
                .collect();
            out.append(&mut mirrors);

            // Remove offsets that fall on the median (divider line)
            out.retain(|v| (v.abs() % lane_width) > 1.0);

            // Sort & dedup
            out.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            out.dedup_by(|a, b| (*a - *b).abs() < 0.1);
            out
        }

        // Collect lane offsets
        let mut ns_x = self.spawn_manager.get_lane_centers(Direction::North, &self.road_config);
        ns_x.extend(self.spawn_manager.get_lane_centers(Direction::South, &self.road_config));

        let mut ew_y = self.spawn_manager.get_lane_centers(Direction::East, &self.road_config);
        ew_y.extend(self.spawn_manager.get_lane_centers(Direction::West, &self.road_config));

        // Filter out medians
        let xs = symmetric_offsets(ns_x, self.road_config.lane_width);
        let ys = symmetric_offsets(ew_y, self.road_config.lane_width);

        // Combine into points
        let mut points = Vec::new();
        for &x in &xs {
            for &y in &ys {
                // Skip exact center (0,0) — no car should target that
                if x.abs() < 0.1 && y.abs() < 0.1 {
                    continue;
                }
                points.push(Vector2::new(x, y));
            }
        }
        points
    }

    fn update_vehicles(&mut self, delta_time: f32) {
        let checkpoints = self.compute_checkpoints();
        let current_time = self.last_update.duration_since(self.start_time).as_secs_f32();

        // Geometry-aware half-width (lanes per direction * lane_width)
        let road_half_width: f32 = self.road_config.lane_width * self.road_config.num_lanes as f32;

        // --- Phase 1: Baseline speed from distance ---
        for v in self.vehicles.iter_mut() {
            if v.state != VehicleState::Moving {
                continue;
            }

            // Use nearest checkpoint for distance-based speed
            let (_, dist) = nearest_checkpoint_distance(v.position, &checkpoints);
            let base_limit = speed_tier_from_distance(dist, road_half_width);
            v.target_speed = base_limit;
        }

        // --- Phase 2: Same-lane following ---
        for i in 0..self.vehicles.len() {
            for j in 0..self.vehicles.len() {
                if i == j {
                    continue;
                }

                let (vehicle1, vehicle2) = if i < j {
                    let (left, right) = self.vehicles.split_at_mut(j);
                    (&mut left[i], &mut right[0])
                } else {
                    let (left, right) = self.vehicles.split_at_mut(i);
                    (&mut right[0], &mut left[j])
                };

                // Check if they're in the same direction and approximately same lane
                if vehicle1.direction == vehicle2.direction {
                    let lane_offset_tolerance = self.road_config.lane_width * 0.5;
                    let same_lane = match vehicle1.direction {
                        Direction::North | Direction::South => {
                            (vehicle1.position.x - vehicle2.position.x).abs() < lane_offset_tolerance
                        }
                        Direction::East | Direction::West => {
                            (vehicle1.position.y - vehicle2.position.y).abs() < lane_offset_tolerance
                        }
                    };

                    if same_lane {
                        let (front_vehicle, rear_vehicle) = 
                            if (vehicle1.position - vehicle2.position).dot(&vehicle1.direction.movement_vector()) > 0.0 {
                                (vehicle1, vehicle2)
                            } else {
                                (vehicle2, vehicle1)
                            };

                        let distance = (rear_vehicle.position - front_vehicle.position).magnitude();
                        let safe_distance = VehiclePhysics::calculate_safety_distance(rear_vehicle.speed);
                        
                        if distance < safe_distance * 1.5 { // Start slowing down before critical distance
                            let safe_speed = front_vehicle.speed * (distance / safe_distance).min(1.0);
                            rear_vehicle.target_speed = rear_vehicle.target_speed.min(safe_speed.max(SPEED_LEVELS[9]));
                        }
                    }
                }
            }
        }

        // --- Phase 3: Intersection conflict resolution ---
        for v in &mut self.vehicles {
            if v.state != VehicleState::Moving {
                continue;
            }

            // Check first conflict waypoint
            if let Some(conflict_point) = v.get_current_waypoint() {
                if (v.position - conflict_point).magnitude() < 100.0 { // Close to conflict
                    let eta = VehiclePhysics::time_to_point(v, conflict_point);
                    match self.conflict_calendar.try_reserve(v.id, conflict_point, current_time + eta, current_time) {
                        Ok(()) => {
                            // Reservation successful - maintain speed
                        }
                        Err(gap) => {
                            // Conflict detected - cap speed based on gap
                            let capped_speed = eta_cap_from_gap(gap);
                            v.target_speed = v.target_speed.min(capped_speed);
                        }
                    }
                }
            }
        }

        // --- Phase 4: Apply hysteresis and integrate physics ---
        for v in &mut self.vehicles {
            if v.state != VehicleState::Moving {
                continue;
            }
            
            // Apply hysteresis to prevent speed flapping
            v.target_speed = quantize_with_hysteresis(v.speed, v.target_speed);

            // Use the physics helper to integrate motion
            VehiclePhysics::update_vehicle_physics(v, delta_time);
        }
    }

    fn cleanup_vehicles(&mut self) {
        let cleanup_distance = 350.0;

        self.vehicles.retain(|vehicle| {
            let distance_from_center = vehicle.position.magnitude();
            distance_from_center <= cleanup_distance
        });
    }

    pub fn render(&mut self, renderer: &mut Renderer) -> Result<(), String> {
        renderer.clear();
        renderer.render_roads(&self.road_config)?;

        // Draw checkpoints in red (now excludes center 0,0)
        let checkpoints = self.compute_checkpoints();
        renderer.render_checkpoints(&checkpoints)?;

        renderer.render_vehicles(&self.vehicles, &self.vehicle_assets)?;
        renderer.present();
        Ok(())
    }
}

/// Helper that doesn't borrow `self`, so it plays nicely with mutable `self.vehicles` iterations.
fn nearest_checkpoint_distance(pos: Vector2<f32>, checkpoints: &[Vector2<f32>]) -> (usize, f32) {
    let mut best_idx = 0usize;
    let mut best = (checkpoints[0] - pos).magnitude();
    for (i, p) in checkpoints.iter().enumerate().skip(1) {
        let d = (*p - pos).magnitude();
        if d < best {
            best = d;
            best_idx = i;
        }
    }
    (best_idx, best)
}
