use std::collections::{HashMap, HashSet};

#[derive(Debug, Default, Clone)]
pub struct StatsSummary {
    pub total_vehicles_passed: u64,
    pub max_speed: Option<f32>,
    pub min_speed: Option<f32>,
    pub max_travel_time: Option<f32>,
    pub min_travel_time: Option<f32>,
    pub close_calls: u64,
    pub collisions: u64,
}

#[derive(Debug, Default, Clone)]
struct VehicleRecord {
    detected_at: Option<f32>,
    max_speed: Option<f32>,
    min_speed: Option<f32>,
}

impl VehicleRecord {
    fn record_speed(&mut self, speed: f32) {
        self.max_speed = Some(self.max_speed.map_or(speed, |current| current.max(speed)));
        self.min_speed = Some(self.min_speed.map_or(speed, |current| current.min(speed)));
    }
}

#[derive(Debug, Default, Clone)]
pub struct StatsManager {
    stats: StatsSummary,
    vehicle_records: HashMap<u64, VehicleRecord>,
    active_close_pairs: HashSet<(u64, u64)>,
    active_collision_pairs: HashSet<(u64, u64)>,
}

impl StatsManager {
    pub fn new() -> Self {
        Self::default()
    }

    fn record_mut(&mut self, vehicle_id: u64) -> &mut VehicleRecord {
        self.vehicle_records
            .entry(vehicle_id)
            .or_insert_with(VehicleRecord::default)
    }

    pub fn observe_speed(&mut self, vehicle_id: u64, speed: f32) {
        let record = self.record_mut(vehicle_id);
        record.record_speed(speed);

        self.stats.max_speed = Some(
            self.stats
                .max_speed
                .map_or(speed, |current| current.max(speed)),
        );
        self.stats.min_speed = Some(
            self.stats
                .min_speed
                .map_or(speed, |current| current.min(speed)),
        );
    }

    pub fn record_detection(&mut self, vehicle_id: u64, detected_at: f32) {
        let record = self.record_mut(vehicle_id);
        record.detected_at.get_or_insert(detected_at);
    }

    pub fn record_vehicle_exit(&mut self, vehicle_id: u64, exit_time: f32) {
        if let Some(record) = self.vehicle_records.remove(&vehicle_id) {
            if let Some(start) = record.detected_at {
                let duration = (exit_time - start).max(0.0);
                self.stats.max_travel_time = Some(
                    self.stats
                        .max_travel_time
                        .map_or(duration, |current| current.max(duration)),
                );
                self.stats.min_travel_time = Some(
                    self.stats
                        .min_travel_time
                        .map_or(duration, |current| current.min(duration)),
                );
            }

            self.stats.total_vehicles_passed += 1;

            self.active_close_pairs
                .retain(|(a, b)| *a != vehicle_id && *b != vehicle_id);
            self.active_collision_pairs
                .retain(|(a, b)| *a != vehicle_id && *b != vehicle_id);
        }
    }

    pub fn observe_distance(
        &mut self,
        id_a: u64,
        id_b: u64,
        distance: f32,
        collision_threshold: f32,
        close_threshold: f32,
    ) {
        let key = if id_a <= id_b {
            (id_a, id_b)
        } else {
            (id_b, id_a)
        };

        if distance < collision_threshold {
            if !self.active_collision_pairs.contains(&key) {
                self.active_collision_pairs.insert(key);
                self.active_close_pairs.insert(key);
                self.stats.collisions += 1;
                self.stats.close_calls += 1;
            }
        } else if distance < close_threshold {
            self.active_collision_pairs.remove(&key);
            if !self.active_close_pairs.contains(&key) {
                self.active_close_pairs.insert(key);
                self.stats.close_calls += 1;
            }
        } else {
            self.active_collision_pairs.remove(&key);
            self.active_close_pairs.remove(&key);
        }
    }

    pub fn summary(&self) -> StatsSummary {
        self.stats.clone()
    }

    pub fn has_results(&self) -> bool {
        self.stats.total_vehicles_passed > 0
            || self.stats.close_calls > 0
            || self.stats.collisions > 0
            || self.stats.max_speed.is_some()
            || self.stats.min_speed.is_some()
            || self.stats.max_travel_time.is_some()
            || self.stats.min_travel_time.is_some()
    }

    pub fn reset(&mut self) {
        self.stats = StatsSummary::default();
        self.vehicle_records.clear();
        self.active_close_pairs.clear();
        self.active_collision_pairs.clear();
    }
}
