use std::f32::consts::PI;

pub const TAU: f32 = 2.0 * PI;

pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

pub fn normalize_angle(angle: f32) -> f32 {
    let mut normalized = angle % TAU;
    if normalized < 0.0 {
        normalized += TAU;
    }
    normalized
}





#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_normalization() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-6);
        assert!((normalize_angle(-PI) - PI).abs() < 1e-6);
    }
}
