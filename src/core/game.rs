use crate::components::{Direction, Vehicle, VehicleState};
use crate::core::{
    geometry::segment_distance, intersection::IntersectionManager, physics, waypoints::Track,
};
use crate::resources::*;
use nalgebra::Vector2;
use rand::{rngs::StdRng, seq::SliceRandom, Rng, SeedableRng};

/// The same fixed-step engine powers both the desktop and headless applications.
pub struct Simulation {
    pub vehicles: Vec<Vehicle>,
    pub tracks: Vec<Track>,
    pub automatic: bool,
    pub time: f64,
    pub intersection: IntersectionManager,
    rng: StdRng,
    stats: StatsManager,
    tick_count: u64,
    next_id: u64,
    last_spawn: [Option<f64>; 4],
    next_auto_spawn: f64,
    spawn_interval: f64,
}

impl Simulation {
    pub fn new(seed: u64, spawn_interval: f64) -> Result<Self, String> {
        if !spawn_interval.is_finite() || !(0.05..=60.0).contains(&spawn_interval) {
            return Err("spawn interval must be between 0.05 and 60 seconds".into());
        }
        let tracks: Vec<Track> = Direction::ALL
            .into_iter()
            .flat_map(|direction| (0..3).map(move |lane| Track::new(direction, lane)))
            .collect();
        let intersection = IntersectionManager::new(&tracks);
        Ok(Self {
            vehicles: Vec::new(),
            tracks,
            automatic: false,
            time: 0.0,
            intersection,
            rng: StdRng::seed_from_u64(seed),
            stats: StatsManager::default(),
            tick_count: 0,
            next_id: 1,
            last_spawn: [None; 4],
            next_auto_spawn: spawn_interval,
            spawn_interval,
        })
    }

    pub fn toggle_automatic(&mut self) {
        self.automatic = !self.automatic;
        self.next_auto_spawn = self.time + self.spawn_interval;
    }

    /// A rejected request never changes the cooldown or moves existing vehicles.
    pub fn spawn(&mut self, direction: Direction) -> bool {
        let index = direction.index();
        if self.vehicles.len() >= MAX_VEHICLES
            || self.last_spawn[index].is_some_and(|last| self.time - last < SPAWN_COOLDOWN)
        {
            self.stats.record_rejection();
            return false;
        }
        let mut lanes = [0, 1, 2];
        lanes.shuffle(&mut self.rng);
        for lane in lanes {
            let track_index = index * 3 + lane;
            let track = &self.tracks[track_index];
            let (position, rotation) = track.pose(0.0);
            if self
                .vehicles
                .iter()
                .any(|v| (v.position - position).norm() < center_separation())
            {
                continue;
            }
            // Different styles have different acceleration and brake response.
            let style = self.rng.gen_range(0..STYLE_COUNT);
            let vehicle = Vehicle {
                id: self.next_id,
                track_index,
                position,
                rotation,
                speed: 0.0,
                progress: 0.0,
                state: VehicleState::Approaching,
                style,
                acceleration: 100.0 + style as f32 * 15.0,
                deceleration: 200.0 + style as f32 * 20.0,
                detected_at: self.time,
                waiting_time: 0.0,
                admitted: false,
            };
            self.vehicles.push(vehicle);
            self.next_id += 1;
            self.last_spawn[index] = Some(self.time);
            self.stats.record_spawn();
            self.stats.observe_speed(0.0);
            return true;
        }
        self.stats.record_rejection();
        false
    }

    pub fn tick(&mut self) {
        self.tick_count += 1;
        self.time = self.tick_count as f64 * FIXED_STEP;
        if self.automatic && self.time + 1e-9 >= self.next_auto_spawn {
            let direction = Direction::ALL[self.rng.gen_range(0..4)];
            self.spawn(direction);
            // Advance the schedule even on rejection; never accumulate a burst.
            self.next_auto_spawn += self.spawn_interval;
        }
        self.intersection.update(&mut self.vehicles, &self.tracks);
        let previous: Vec<Vector2<f32>> = self.vehicles.iter().map(|v| v.position).collect();
        physics::update_vehicles(&mut self.vehicles, &self.tracks, FIXED_STEP as f32);
        for i in 0..self.vehicles.len() {
            self.stats.observe_speed(self.vehicles[i].speed);
            for j in (i + 1)..self.vehicles.len() {
                // Relative swept motion detects violations between ticks, too.
                let before = previous[i] - previous[j];
                let after = self.vehicles[i].position - self.vehicles[j].position;
                let distance = segment_distance(before, after, Vector2::zeros(), Vector2::zeros());
                self.stats.observe_distance(
                    self.vehicles[i].id,
                    self.vehicles[j].id,
                    distance,
                    vehicle_radius() * 2.0,
                    center_separation(),
                );
            }
        }
        let waiting = self
            .vehicles
            .iter()
            .filter(|v| v.state == VehicleState::Waiting)
            .count();
        self.stats
            .observe_load(waiting, self.intersection.active_count());
        self.vehicles.retain(|v| {
            if v.progress >= self.tracks[v.track_index].length - 0.01 {
                self.stats.record_exit(v, self.time);
                false
            } else {
                true
            }
        });
    }

    pub fn summary(&self) -> StatsSummary {
        self.stats.summary(self.time, self.vehicles.len())
    }
}
