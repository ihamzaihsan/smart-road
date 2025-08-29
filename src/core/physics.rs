use crate::components::Vehicle;
use nalgebra::Vector2;

pub const MIN_SAFETY_DISTANCE: f32 = 60.0;
pub const MAX_ACCELERATION: f32 = 220.0;
pub const MAX_DECELERATION: f32 = 100.0;

/// Exactly 4 discrete speeds (fast → slow)
pub const SPEED_LEVELS: [f32; 4] = [400.0, 240.0, 180.0, 120.0];

/// Snap any speed to the nearest allowed level (and clamp to >= 0).
#[inline]
pub fn quantize_speed(speed: f32) -> f32 {
    let mut closest = SPEED_LEVELS[0];
    let mut best = (speed - closest).abs();
    for &lvl in &SPEED_LEVELS[1..] {
        let d = (speed - lvl).abs();
        if d < best {
            best = d;
            closest = lvl;
        }
    }
    if closest < 0.0 { 0.0 } else { closest }
}

/// Distance-based baseline tier tied to intersection size.
/// `road_half_width` = lane_width * lanes_per_direction (half the total crossing).
#[inline]
pub fn speed_tier_from_distance(dist: f32, road_half_width: f32) -> f32 {
   
    if dist >= 2.0 * road_half_width {
        SPEED_LEVELS[0] 
    } else if dist <= 1.0 * road_half_width {
        SPEED_LEVELS[1] 
    } else if dist <= 0.5 * road_half_width {
        SPEED_LEVELS[2] 
    } else {
        SPEED_LEVELS[3] 
    }
}

pub struct VehiclePhysics;

impl VehiclePhysics {
    /// Safety distance increases with speed.
    pub fn calculate_safety_distance(speed: f32) -> f32 {
        MIN_SAFETY_DISTANCE + (speed * 0.6)
    }

    /// Integrate vehicle motion toward `vehicle.target_speed` (already chosen by game logic).
    pub fn update_vehicle_physics(vehicle: &mut Vehicle, delta_time: f32) {
        // Quantize both current and target speeds to the 4-level policy
        vehicle.speed = quantize_speed(vehicle.speed);
        vehicle.target_speed = quantize_speed(vehicle.target_speed);

        // Accelerate / decelerate toward target
        let speed_diff = vehicle.target_speed - vehicle.speed;
        let acceleration = if speed_diff > 0.0 {
            MAX_ACCELERATION
        } else if speed_diff < 0.0 {
            -MAX_DECELERATION
        } else {
            0.0
        };

        // Integrate speed with time step and clamp between 0 and target
        vehicle.speed += acceleration * delta_time;
        if speed_diff >= 0.0 {
            if vehicle.speed > vehicle.target_speed {
                vehicle.speed = vehicle.target_speed;
            }
        } else {
            if vehicle.speed < vehicle.target_speed {
                vehicle.speed = vehicle.target_speed;
            }
        }

        // Move along the direction vector
        let movement = vehicle.direction.movement_vector() * vehicle.speed * delta_time;
        vehicle.position += movement;
    }

    /// Check if two vehicles are at a safe distance given their speeds.
    pub fn check_safe_distance(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let distance = (vehicle1.position - vehicle2.position).magnitude();
        let required_distance = Self::calculate_safety_distance(vehicle1.speed.max(vehicle2.speed));
        distance >= required_distance
    }

    /// Utility: compute ETA (time to reach a point) using current speed or a fallback.
    /// If the vehicle is practically stopped, use its target speed for ETA.
    pub fn time_to_point(vehicle: &Vehicle, point: Vector2<f32>) -> f32 {
        let dist = (point - vehicle.position).magnitude();
        let v = if vehicle.speed > 1.0 { vehicle.speed } else { vehicle.target_speed.max(1.0) };
        dist / v
    }
}
