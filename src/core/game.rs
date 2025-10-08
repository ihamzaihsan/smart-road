use crate::components::{Direction, Vehicle};
use crate::core::physics::ConflictCalendar;
use crate::core::{intersection, spawning, vehicle_update, waypoints};
use crate::input::InputSystem;
use crate::rendering::Renderer;
use crate::resources::{RoadConfig, SimulationConfig, StatsManager, StatsSummary, VehicleAssets};
use crate::systems::{AutoSpawnTimer, SpawnManager};
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
    current_intersection_users: Vec<intersection::ActiveIntersectionUser>,
    stats_manager: StatsManager,
    pending_exit: bool,
    stats_ready: bool,
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
            stats_manager: StatsManager::new(),
            pending_exit: false,
            stats_ready: false,
        })
    }

    pub fn update(&mut self, events: &[Event]) -> bool {
        let current_time = Instant::now();
        let delta_time = current_time.duration_since(self.last_update).as_secs_f32();

        self.input_system.update(events);

        let mut exit_requested = false;
        for event in events {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => exit_requested = true,
                _ => {}
            }
        }

        if exit_requested {
            self.pending_exit = true;
            self.auto_spawn_timer.enabled = false;
        }

        // Handle spawning
        self.handle_spawning(current_time);

        // Update vehicles
        let elapsed_seconds = current_time.duration_since(self.start_time).as_secs_f32();
        self.update_all_vehicles(delta_time, elapsed_seconds);

        self.last_update = current_time;
        self.cleanup_vehicles(elapsed_seconds);

        for vehicle in &self.vehicles {
            self.stats_manager.observe_speed(vehicle.id, vehicle.speed);
        }

        let no_active_traffic =
            self.vehicles.is_empty() && self.current_intersection_users.is_empty();

        if no_active_traffic && self.stats_manager.has_results() && !self.stats_ready {
            self.stats_ready = true;
        }

        // Allow immediate exit when requested
        if self.pending_exit {
            return false;
        }

        true
    }

    fn handle_spawning(&mut self, current_time: Instant) {
        if self.pending_exit {
            return;
        }

        // Handle keyboard spawning
        if let Some((direction, lane_index, spawn_pos, route)) = spawning::handle_keyboard_spawning(
            &self.input_system,
            &mut self.spawn_manager,
            &self.vehicles,
            &self.road_config,
            &mut self.last_lane_used,
            current_time,
            self.start_time,
        ) {
            let path = waypoints::generate_waypoints(
                direction,
                lane_index,
                route,
                spawn_pos,
                &self.road_config,
                &self.simulation_config,
                &self.spawn_manager,
            );
            let vehicle = spawning::spawn_vehicle_at_position(
                direction,
                lane_index,
                spawn_pos,
                route,
                path,
                &mut self.spawn_manager,
                &self.vehicle_assets,
            );
            self.stats_manager.observe_speed(vehicle.id, vehicle.speed);
            self.vehicles.push(vehicle);
        }

        // Handle spawn toggle
        spawning::handle_spawn_toggle(
            &self.input_system,
            &mut self.auto_spawn_timer,
            &mut self.last_auto_spawn_at,
        );

        // Handle automatic spawning
        if self.auto_spawn_timer.enabled {
            if let Some((direction, lane_index, spawn_pos, route)) =
                spawning::handle_automatic_spawning(
                    &mut self.auto_spawn_timer,
                    &mut self.spawn_manager,
                    &self.vehicles,
                    &self.road_config,
                    &self.simulation_config,
                    &mut self.last_lane_used,
                    current_time,
                    self.start_time,
                    self.last_auto_spawn_at,
                    self.min_auto_spawn_gap,
                )
            {
                let path = waypoints::generate_waypoints(
                    direction,
                    lane_index,
                    route,
                    spawn_pos,
                    &self.road_config,
                    &self.simulation_config,
                    &self.spawn_manager,
                );
                let vehicle = spawning::spawn_vehicle_at_position(
                    direction,
                    lane_index,
                    spawn_pos,
                    route,
                    path,
                    &mut self.spawn_manager,
                    &self.vehicle_assets,
                );
                self.stats_manager.observe_speed(vehicle.id, vehicle.speed);
                self.vehicles.push(vehicle);
                self.last_auto_spawn_at = current_time;
            }
        }
    }

    fn update_all_vehicles(&mut self, delta_time: f32, current_time: f32) {
        // Priority-based intersection control
        intersection::apply_priority_based_intersection_control(
            &mut self.vehicles,
            &mut self.current_intersection_users,
            &self.road_config,
            &self.spawn_manager,
            delta_time,
            current_time,
            &mut self.stats_manager,
        );

        // Update vehicle physics and collision avoidance
        vehicle_update::update_vehicles(
            &mut self.vehicles,
            &mut self.conflict_calendar,
            &self.road_config,
            delta_time,
            current_time,
            &mut self.stats_manager,
        );
    }

    fn cleanup_vehicles(&mut self, current_time: f32) {
        let exited = vehicle_update::cleanup_vehicles(
            &mut self.vehicles,
            self.simulation_config.cleanup_distance,
        );
        for id in exited {
            self.stats_manager.record_vehicle_exit(id, current_time);
        }
    }

    pub fn render(&mut self, renderer: &mut Renderer) -> Result<(), String> {
        renderer.clear();
        renderer.render_roads(&self.road_config)?;
        renderer.render_vehicles(&self.vehicles, &self.vehicle_assets)?;
        renderer.present();
        Ok(())
    }

    pub fn take_stats_summary(&mut self) -> Option<StatsSummary> {
        if self.stats_ready && self.stats_manager.has_results() {
            self.stats_ready = false;
            let summary = self.stats_manager.summary();
            self.stats_manager.reset();
            Some(summary)
        } else {
            None
        }
    }
}
