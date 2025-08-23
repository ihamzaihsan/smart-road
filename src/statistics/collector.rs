use std::time::Instant;

#[derive(Debug, Default, Clone)]
pub struct SimulationStats {
    pub vehicles_passed: u64,
    pub max_speed: f32,
    pub min_speed: f32,
    pub max_time: f32,
    pub min_time: f32,
    pub close_calls: u64,
    pub start_time: Option<Instant>,
}

impl SimulationStats {
    pub fn new() -> Self {
        Self {
            start_time: Some(Instant::now()),
            ..Default::default()
        }
    }

    pub fn record_speed(&mut self, speed: f32) {
        if self.vehicles_passed == 0 {
            self.max_speed = speed;
            self.min_speed = speed;
        } else {
            if speed > self.max_speed {
                self.max_speed = speed;
            }
            if speed < self.min_speed {
                self.min_speed = speed;
            }
        }
    }

    pub fn record_passage_time(&mut self, time: f32) {
        if self.vehicles_passed == 0 {
            self.max_time = time;
            self.min_time = time;
        } else {
            if time > self.max_time {
                self.max_time = time;
            }
            if time < self.min_time {
                self.min_time = time;
            }
        }
        self.vehicles_passed += 1;
    }

    pub fn record_close_call(&mut self) {
        self.close_calls += 1;
    }

    pub fn get_total_time(&self) -> f32 {
        if let Some(start) = self.start_time {
            start.elapsed().as_secs_f32()
        } else {
            0.0
        }
    }
}

pub type Stats = SimulationStats;