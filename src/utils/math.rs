use std::f32::consts::PI;

pub const TAU: f32 = 2.0 * PI;

pub fn deg_to_rad(degrees: f32) -> f32 {
    degrees * PI / 180.0
}

pub fn rad_to_deg(radians: f32) -> f32 {
    radians * 180.0 / PI
}

pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

pub fn normalize_angle(angle: f32) -> f32 {
    let mut normalized = angle % TAU;
    if normalized < 0.0 {
        normalized += TAU;
    }
    normalized
}

pub fn angle_difference(a: f32, b: f32) -> f32 {
    let diff = normalize_angle(b - a);
    if diff > PI {
        diff - TAU
    } else {
        diff
    }
}

pub struct MovingAverage {
    values: Vec<f32>,
    capacity: usize,
    sum: f32,
}

impl MovingAverage {
    pub fn new(capacity: usize) -> Self {
        Self {
            values: Vec::with_capacity(capacity),
            capacity,
            sum: 0.0,
        }
    }

    pub fn add(&mut self, value: f32) {
        if self.values.len() == self.capacity {
            let old_value = self.values.remove(0);
            self.sum -= old_value;
        }
        self.values.push(value);
        self.sum += value;
    }

    pub fn average(&self) -> f32 {
        if self.values.is_empty() {
            0.0
        } else {
            self.sum / self.values.len() as f32
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deg_rad_conversion() {
        assert!((deg_to_rad(180.0) - PI).abs() < 1e-6);
        assert!((rad_to_deg(PI) - 180.0).abs() < 1e-6);
    }

    #[test]
    fn angle_normalization() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-6);
        assert!((normalize_angle(-PI) - PI).abs() < 1e-6);
    }

    #[test]
    fn moving_average() {
        let mut avg = MovingAverage::new(3);
        avg.add(1.0);
        avg.add(2.0);
        avg.add(3.0);
        assert!((avg.average() - 2.0).abs() < 1e-6);
        
        avg.add(4.0);
        assert!((avg.average() - 3.0).abs() < 1e-6);
    }
}