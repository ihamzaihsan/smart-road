use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::render::TextureCreator;
use sdl2::video::WindowContext;
use sdl2::image::LoadTexture;
use std::time::Instant;
use nalgebra::Vector2;

use crate::components::{Vehicle, Direction, VehicleState, Route};
use crate::resources::{RoadConfig, SpawnManager, AutoSpawnTimer, VehicleAssets};
use crate::rendering::Renderer;
use crate::input::InputSystem;

/// Main game state and logic
pub struct Game<'a> {
    pub vehicles: Vec<Vehicle>,
    pub spawn_manager: SpawnManager,
    pub auto_spawn_timer: AutoSpawnTimer,
    pub road_config: RoadConfig,
    pub vehicle_assets: VehicleAssets<'a>,
    pub input_system: InputSystem,
    pub last_update: Instant,
    pub start_time: Instant,  // Track program start time for consistent timing
}

impl<'a> Game<'a> {
    /// Create a new game instance
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
        })
    }

    /// Update game logic
    pub fn update(&mut self, events: &[Event]) -> bool {
        let current_time = Instant::now();
        let delta_time = current_time.duration_since(self.last_update).as_secs_f32();
        self.last_update = current_time;

        // Debug output removed to avoid static mut warning

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

        // Handle automatic spawning
        if self.auto_spawn_timer.enabled {
            self.handle_automatic_spawning(current_time);
        }

        // Update vehicle movement
        self.update_vehicles(delta_time);

        // Clean up vehicles that have left the screen
        self.cleanup_vehicles();

        true // Continue running
    }

    /// Handle keyboard-controlled vehicle spawning
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

    /// Handle R key toggle for automatic spawning
    fn handle_spawn_toggle(&mut self) {
        if self.input_system.just_pressed(Keycode::R) {
            self.auto_spawn_timer.enabled = !self.auto_spawn_timer.enabled;

            if self.auto_spawn_timer.enabled {
                println!("🚗 Automatic spawning ENABLED - vehicles will spawn every 0.3 seconds");
                self.auto_spawn_timer.reset_timer();
            } else {
                println!("⏸️  Automatic spawning DISABLED - use arrow keys for manual spawning");
            }
        }
    }

    /// Handle automatic vehicle spawning
    fn handle_automatic_spawning(&mut self, current_time: Instant) {
        if !self.auto_spawn_timer.should_spawn() {
            return;
        }

        let current_time_f64 = current_time.duration_since(self.start_time).as_secs_f64();
        let direction = AutoSpawnTimer::random_direction();

        if self.spawn_manager.can_spawn(direction, current_time_f64) {
            if let Some(spawn_pos) = self.find_safe_spawn_position(direction) {
                self.spawn_vehicle_at_position(direction, spawn_pos);
                self.spawn_manager.record_spawn(direction, current_time_f64);
                self.auto_spawn_timer.reset_timer(); // Only reset timer when spawn succeeds
            }
        }
    }

    /// Find a safe spawn position using fixed lane selection (for keyboard input)
    fn find_safe_spawn_position_fixed_lane(&self, direction: Direction) -> Option<Vector2<f32>> {
        let lane_centers = self.spawn_manager.get_lane_centers(direction, &self.road_config);
        
        // Try lanes in order (fixed positioning for keyboard input)
        for &lane_offset in &lane_centers {
            let potential_pos = self.spawn_manager.get_spawn_position(direction, lane_offset);
            
            // Check if this position is safe (no vehicles too close)
            let is_safe = self.vehicles.iter().all(|vehicle| {
                let distance = (potential_pos - vehicle.position).magnitude();
                distance >= self.spawn_manager.min_spawn_separation
            });
            
            if is_safe {
                return Some(potential_pos);
            }
        }
        
        None // No safe position found
    }

    /// Find a safe spawn position using random lane selection (for automatic spawning)
    fn find_safe_spawn_position(&self, direction: Direction) -> Option<Vector2<f32>> {
        let lane_centers = self.spawn_manager.get_lane_centers(direction, &self.road_config);
        
        // First try a random lane
        let random_lane_offset = self.spawn_manager.get_random_lane_center(direction, &self.road_config);
        let potential_pos = self.spawn_manager.get_spawn_position(direction, random_lane_offset);
        
        // Check if this position is safe (no vehicles too close)
        let is_safe = self.vehicles.iter().all(|vehicle| {
            let distance = (potential_pos - vehicle.position).magnitude();
            distance >= self.spawn_manager.min_spawn_separation
        });
        
        if is_safe {
            return Some(potential_pos);
        }
        
        // If random lane is not safe, try all other lanes as fallback
        for &lane_offset in &lane_centers {
            if (lane_offset - random_lane_offset).abs() < 0.1 {
                continue; // Skip the random lane we already tried
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
        
        None // No safe position found
    }

    /// Spawn a vehicle at the specified position
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
            target_speed: 100.0, // pixels per second
            rotation,
            texture_index,
        };

        self.vehicles.push(vehicle);
    }

    /// Update vehicle movement and physics
    fn update_vehicles(&mut self, delta_time: f32) {
        for vehicle in &mut self.vehicles {
            if vehicle.state != VehicleState::Moving {
                continue;
            }

            // Gradually accelerate to target speed
            let speed_diff = vehicle.target_speed - vehicle.speed;
            let acceleration = 50.0; // units per second squared
            vehicle.speed += speed_diff.signum() * acceleration * delta_time;
            vehicle.speed = vehicle.speed.clamp(0.0, vehicle.target_speed);

            // Move the vehicle
            let movement = vehicle.direction.movement_vector() * vehicle.speed * delta_time;
            vehicle.position += movement;
        }
    }

    /// Remove vehicles that have left the simulation area
    fn cleanup_vehicles(&mut self) {
        let cleanup_distance = 350.0; // Reduced distance for more aggressive cleanup


        self.vehicles.retain(|vehicle| {
            let distance_from_center = vehicle.position.magnitude();
            distance_from_center <= cleanup_distance
        });

        // Vehicles are cleaned up silently
    }

    /// Render the game
    pub fn render(&mut self, renderer: &mut Renderer) -> Result<(), String> {
        renderer.clear();
        renderer.render_roads(&self.road_config)?;
        renderer.render_vehicles(&self.vehicles, &self.vehicle_assets)?;
        renderer.present();
        Ok(())
    }
}
