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
    VehiclePhysics, quantize_speed, speed_tier_from_distance, SPEED_LEVELS,
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

    // Throttle random auto-spawn by +1s (on top of timer’s own cadence)
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
                if let Some(spawn_pos) = self.find_safe_spawn_position_fixed_lane(direction) {
                    self.spawn_vehicle_at_position(direction, spawn_pos);
                    self.spawn_manager.record_spawn(direction, current_time_f64);
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
        // Add a +1s gap on top of the AutoSpawnTimer cadence
        if !self.auto_spawn_timer.should_spawn() {
            return;
        }
        if current_time.duration_since(self.last_auto_spawn_at) < self.min_auto_spawn_gap {
            return;
        }

        let current_time_f64 = current_time.duration_since(self.start_time).as_secs_f64();
        let direction = AutoSpawnTimer::random_direction();

        if self.spawn_manager.can_spawn(direction, current_time_f64) {
            if let Some(spawn_pos) = self.find_safe_spawn_position(direction) {
                self.spawn_vehicle_at_position(direction, spawn_pos);
                self.spawn_manager.record_spawn(direction, current_time_f64);
                self.auto_spawn_timer.reset_timer();
                self.last_auto_spawn_at = current_time;
            }
        }
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


    fn find_safe_spawn_position_fixed_lane(&self, direction: Direction) -> Option<Vector2<f32>> {
        let lane_centers = self.spawn_manager.get_lane_centers(direction, &self.road_config);
        
        for &lane_offset in &lane_centers {
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

        let vehicle = Vehicle {
            id: vehicle_id,
            position,
            speed: 0.0,
            state: VehicleState::Moving,
            route: Route::Straight,
            direction,
            // Start by aiming for the fastest tier; checkpoint logic will cap this each frame.
            target_speed: SPEED_LEVELS[0], // 200
            rotation,
            texture_index,
        };

        self.vehicles.push(vehicle);
    }

    fn update_vehicles(&mut self, delta_time: f32) {
        let checkpoints = self.compute_checkpoints();

        // Geometry-aware half-width (lanes per direction * lane_width)
        let road_half_width: f32 = self.road_config.lane_width * self.road_config.num_lanes as f32;

        // --- Phase 1: Baseline speed + collect per-checkpoint approachers ---
        #[derive(Clone)]
        struct Approaching {
            veh_idx: usize,
            checkpoint_idx: usize,
            eta: f32,
            dist: f32,
        }

        let mut by_checkpoint: Vec<Vec<Approaching>> = vec![Vec::new(); checkpoints.len()];

        for (i, v) in self.vehicles.iter_mut().enumerate() {
            if v.state != VehicleState::Moving {
                continue;
            }

            // Nearest checkpoint and distance (no &self borrow here)
            let (cp_idx, dist) = nearest_checkpoint_distance(v.position, &checkpoints);

            // Baseline tier from distance (geometry-aware)
            let base_limit = speed_tier_from_distance(dist, road_half_width);
            if v.target_speed > base_limit {
                v.target_speed = base_limit;
            }

            // Within detection radius? consider for conflict resolution
            const DETECT_RADIUS: f32 = 420.0;
            if dist <= DETECT_RADIUS {
                let eta = VehiclePhysics::time_to_point(v, checkpoints[cp_idx]);
                by_checkpoint[cp_idx].push(Approaching { veh_idx: i, checkpoint_idx: cp_idx, eta, dist });
            }
        }

        // --- Phase 2: Conflict resolution per checkpoint ---
        // Reservation model: at most ONE “leader” (passes fast), others yield (slow/medium).
        for mut group in by_checkpoint.into_iter() {
            if group.is_empty() { continue; }

            group.sort_by(|a, b| {
                a.eta
                    .partial_cmp(&b.eta)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| a.dist.partial_cmp(&b.dist).unwrap_or(std::cmp::Ordering::Equal))
            });

            // Choose leader if arrivals are within a window
            const ETA_WINDOW: f32 = 1.2; // seconds
            const LEADER_DISTANCE_MULTIPLIER: f32 = 1.2;
             let leader = &group.clone()[0];
            // Leader speed: 600 if still outside ~1.2×half-width, otherwise 440
            let leader_speed = if leader.dist >= LEADER_DISTANCE_MULTIPLIER * road_half_width { SPEED_LEVELS[0] } else { SPEED_LEVELS[2] };
            self.vehicles[leader.veh_idx].target_speed = leader_speed;

            // Everyone else yields: use 200 if very tight, else 360
            for follower in group.into_iter().skip(1) {
                let dt = (follower.eta - leader.eta).abs();
                let desired = if dt <= ETA_WINDOW { SPEED_LEVELS[5] } else { SPEED_LEVELS[3] };
                // Also cap by distance policy
                let cap = speed_tier_from_distance(follower.dist, road_half_width);
                self.vehicles[follower.veh_idx].target_speed = desired.min(cap);
            }
        }

        // --- Phase 3: Integrate physics ---
        for v in &mut self.vehicles {
            if v.state != VehicleState::Moving {
                continue;
            }
            // Quantize before integrating (keeps behavior stable)
            v.target_speed = quantize_speed(v.target_speed);
            v.speed = quantize_speed(v.speed);

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