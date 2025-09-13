#[derive(Debug, Default, Clone)]
pub struct Stats {
    pub vehicles_passed: u64,
    pub max_speed: f32,
    pub min_speed: f32,
    pub close_calls: u64,
    pub collisions: u64,
}

impl Stats {
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
        self.vehicles_passed += 1;
    }
}