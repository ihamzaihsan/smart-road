use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::render::TextureCreator;
use sdl2::video::WindowContext;
use sdl2::image::LoadTexture;
use std::time::Instant;
use nalgebra::Vector2;

use crate::components::{Vehicle, Direction, VehicleState, Route};
use crate::resources::{RoadConfig, VehicleAssets};
use crate::systems::{SpawnManager, AutoSpawnTimer};
use crate::rendering::Renderer;
use crate::input::InputSystem;

pub struct Game<'a> {
    pub vehicles: Vec<Vehicle>,
    pub spawn_manager: SpawnManager,
    pub auto_spawn_timer: AutoSpawnTimer,
    pub road_config: RoadConfig,
    pub vehicle_assets: VehicleAssets<'a>,
    pub input_system: InputSystem,
    pub last_update: Instant,
    pub start_time: Instant,  
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
                println!("🚗 Automatic spawning ENABLED - vehicles will spawn every 0.3 seconds");
                self.auto_spawn_timer.reset_timer();
            } else {
                println!("⏸️  Automatic spawning DISABLED - use arrow keys for manual spawning");
            }
        }
    }

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
                self.auto_spawn_timer.reset_timer(); 
            }
        }
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
            // Start by aiming for the fastest tier; checkpoints will cap it as needed.
            target_speed: 200.0,
            rotation,
            texture_index,
        };

        self.vehicles.push(vehicle);
    }

    fn update_vehicles(&mut self, delta_time: f32) {
        use crate::core::physics::{
            checkpoint_speed_limit, quantize_speed, MAX_ACCELERATION, MAX_DECELERATION,
        };

        for vehicle in &mut self.vehicles {
            if vehicle.state != VehicleState::Moving {
                continue;
            }

            // Checkpoint speed policy (reduces crashes near the center)
            let dist = vehicle.position.magnitude();
            let limit = checkpoint_speed_limit(dist);
            if vehicle.target_speed > limit {
                vehicle.target_speed = limit;
            }

            // Quantize speeds to the 10 allowed levels (200 -> 40)
            vehicle.speed = quantize_speed(vehicle.speed);
            vehicle.target_speed = quantize_speed(vehicle.target_speed);

            // Accelerate/decelerate toward target speed
            let speed_diff = vehicle.target_speed - vehicle.speed;
            let accel = if speed_diff > 0.0 {
                MAX_ACCELERATION
            } else if speed_diff < 0.0 {
                -MAX_DECELERATION
            } else {
                0.0
            };

            vehicle.speed += accel * delta_time;
            if speed_diff >= 0.0 {
                if vehicle.speed > vehicle.target_speed {
                    vehicle.speed = vehicle.target_speed;
                }
            } else {
                if vehicle.speed < vehicle.target_speed {
                    vehicle.speed = vehicle.target_speed;
                }
            }

            // Move the vehicle
            let movement = vehicle.direction.movement_vector() * vehicle.speed * delta_time;
            vehicle.position += movement;
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
        renderer.render_vehicles(&self.vehicles, &self.vehicle_assets)?;
        renderer.present();
        Ok(())
    }
}
