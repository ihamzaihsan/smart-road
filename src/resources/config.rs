/// Simulation units are pixels and seconds, not calibrated road measurements.
pub const LANE_WIDTH: f32 = 50.0;
pub const INTERSECTION_HALF: f32 = LANE_WIDTH * 3.0;
pub const VEHICLE_WIDTH: f32 = 16.0;
pub const VEHICLE_LENGTH: f32 = 32.0;
pub const SAFETY_GAP: f32 = 10.0;
pub const FIXED_STEP: f64 = 1.0 / 120.0;
pub const SPAWN_COOLDOWN: f64 = 0.5;
pub const MAX_VEHICLES: usize = 60;
pub const CRUISE_SPEED: f32 = 180.0;
pub const CROSSING_SPEED: f32 = 140.0;
pub const TURN_SPEED: f32 = 90.0;
pub const STYLE_COUNT: usize = 5;

/// A circle enclosing the full sprite protects its corners throughout a turn.
pub fn vehicle_radius() -> f32 {
    (VEHICLE_WIDTH * VEHICLE_WIDTH + VEHICLE_LENGTH * VEHICLE_LENGTH).sqrt() * 0.5
}

pub fn center_separation() -> f32 {
    vehicle_radius() * 2.0 + SAFETY_GAP
}
