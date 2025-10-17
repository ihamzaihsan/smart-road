use crate::components::Vehicle;
use std::collections::HashSet;
use std::fmt;

#[derive(Debug, Default, Clone)]
pub struct StatsSummary {
    pub spawned: u64,
    pub rejected_spawns: u64,
    pub total_vehicles_passed: u64,
    pub active_vehicles: usize,
    pub elapsed_seconds: f64,
    pub max_speed: Option<f32>,
    pub min_speed: Option<f32>,
    pub max_travel_time: Option<f64>,
    pub min_travel_time: Option<f64>,
    pub mean_travel_time: Option<f64>,
    pub mean_waiting_time: Option<f64>,
    pub throughput_per_minute: f64,
    pub completed_distance: f64,
    pub peak_queue: usize,
    pub peak_concurrent_crossings: usize,
    pub close_calls: u64,
    pub collisions: u64,
}

fn value(number: Option<f64>) -> String {
    number
        .map(|v| format!("{v:.2}"))
        .unwrap_or_else(|| "N/A".into())
}

impl fmt::Display for StatsSummary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f,
            "Elapsed time: {:.2} s\nSpawned: {}\nCompleted crossings: {}\nActive vehicles: {}\nRejected spawn requests: {}\nMax speed: {} px/s\nMin speed (including stops): {} px/s\nLongest travel time: {} s\nShortest travel time: {} s\nMean travel time: {} s\nMean stopped waiting time: {} s\nThroughput: {:.2} vehicles/min\nCompleted travel distance: {:.2} px\nPeak stopped queue: {}\nPeak concurrent reservations: {}\nClose calls: {}\nCollisions: {}",
            self.elapsed_seconds, self.spawned, self.total_vehicles_passed,
            self.active_vehicles, self.rejected_spawns,
            value(self.max_speed.map(f64::from)), value(self.min_speed.map(f64::from)),
            value(self.max_travel_time), value(self.min_travel_time),
            value(self.mean_travel_time), value(self.mean_waiting_time),
            self.throughput_per_minute, self.completed_distance, self.peak_queue,
            self.peak_concurrent_crossings, self.close_calls, self.collisions)
    }
}

impl StatsSummary {
    /// Stable, two-column CSV for spreadsheet comparison of seeded scenarios.
    pub fn csv(&self) -> String {
        let optional = |v: Option<f64>| v.map(|n| format!("{n:.6}")).unwrap_or_default();
        format!(
            "metric,value\nelapsed_seconds,{:.6}\nspawned,{}\ncompleted,{}\nactive,{}\nrejected_spawns,{}\nmax_speed_px_s,{}\nmin_speed_px_s,{}\nmax_travel_seconds,{}\nmin_travel_seconds,{}\nmean_travel_seconds,{}\nmean_waiting_seconds,{}\nthroughput_per_minute,{:.6}\ncompleted_distance_px,{:.6}\npeak_queue,{}\npeak_concurrent_reservations,{}\nclose_calls,{}\ncollisions,{}\n",
            self.elapsed_seconds, self.spawned, self.total_vehicles_passed,
            self.active_vehicles, self.rejected_spawns,
            optional(self.max_speed.map(f64::from)), optional(self.min_speed.map(f64::from)),
            optional(self.max_travel_time), optional(self.min_travel_time),
            optional(self.mean_travel_time), optional(self.mean_waiting_time),
            self.throughput_per_minute, self.completed_distance, self.peak_queue,
            self.peak_concurrent_crossings, self.close_calls, self.collisions)
    }
}

#[derive(Default)]
pub struct StatsManager {
    stats: StatsSummary,
    total_travel_time: f64,
    total_waiting_time: f64,
    active_close_pairs: HashSet<(u64, u64)>,
    active_collision_pairs: HashSet<(u64, u64)>,
}

impl StatsManager {
    pub fn record_spawn(&mut self) {
        self.stats.spawned += 1;
    }
    pub fn record_rejection(&mut self) {
        self.stats.rejected_spawns += 1;
    }

    pub fn observe_speed(&mut self, speed: f32) {
        self.stats.max_speed = Some(self.stats.max_speed.map_or(speed, |v| v.max(speed)));
        self.stats.min_speed = Some(self.stats.min_speed.map_or(speed, |v| v.min(speed)));
    }

    pub fn observe_load(&mut self, waiting: usize, concurrent: usize) {
        self.stats.peak_queue = self.stats.peak_queue.max(waiting);
        self.stats.peak_concurrent_crossings = self.stats.peak_concurrent_crossings.max(concurrent);
    }

    pub fn record_exit(&mut self, vehicle: &Vehicle, now: f64) {
        let duration = now - vehicle.detected_at;
        self.stats.total_vehicles_passed += 1;
        self.total_travel_time += duration;
        self.total_waiting_time += vehicle.waiting_time;
        self.stats.completed_distance += vehicle.progress as f64;
        self.stats.max_travel_time = Some(
            self.stats
                .max_travel_time
                .map_or(duration, |v| v.max(duration)),
        );
        self.stats.min_travel_time = Some(
            self.stats
                .min_travel_time
                .map_or(duration, |v| v.min(duration)),
        );
        self.active_close_pairs
            .retain(|&(a, b)| a != vehicle.id && b != vehicle.id);
        self.active_collision_pairs
            .retain(|&(a, b)| a != vehicle.id && b != vehicle.id);
    }

    /// Count entries into a safety violation, rather than counting every frame.
    pub fn observe_distance(&mut self, a: u64, b: u64, distance: f32, collision: f32, close: f32) {
        let pair = (a.min(b), a.max(b));
        if distance < close - 0.01 {
            if self.active_close_pairs.insert(pair) {
                self.stats.close_calls += 1;
            }
        } else {
            self.active_close_pairs.remove(&pair);
        }
        if distance < collision {
            if self.active_collision_pairs.insert(pair) {
                self.stats.collisions += 1;
            }
        } else {
            self.active_collision_pairs.remove(&pair);
        }
    }

    pub fn summary(&self, elapsed: f64, active: usize) -> StatsSummary {
        let mut result = self.stats.clone();
        result.elapsed_seconds = elapsed;
        result.active_vehicles = active;
        if result.total_vehicles_passed > 0 {
            let count = result.total_vehicles_passed as f64;
            result.mean_travel_time = Some(self.total_travel_time / count);
            result.mean_waiting_time = Some(self.total_waiting_time / count);
        }
        if elapsed > 0.0 {
            result.throughput_per_minute = result.total_vehicles_passed as f64 * 60.0 / elapsed;
        }
        result
    }
}
