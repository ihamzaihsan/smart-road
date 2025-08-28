use crate::components::Vehicle;

pub const MIN_SAFETY_DISTANCE: f32 = 60.0;
pub const MAX_ACCELERATION: f32 = 200.0;
pub const MAX_DECELERATION: f32 = 80.0;

// === Discrete speed policy ===
// Ten speed levels linearly spaced from 200 down to 40 (inclusive).
// You can change these if you want a different ladder.
pub const SPEED_LEVELS: [f32; 10] = [
    200.0, 182.2222, 164.4444, 146.6667, 128.8889,
    111.1111, 93.3333, 75.5556, 57.7778, 40.0,
];

/// Snap any speed to the nearest allowed level (and clamp to >= 0).
#[inline]
pub fn quantize_speed(speed: f32) -> f32 {
    let mut closest = SPEED_LEVELS[0];
    let mut best = (speed - closest).abs();
    let mut i = 1usize;
    while i < SPEED_LEVELS.len() {
        let d = (speed - SPEED_LEVELS[i]).abs();
        if d < best {
            best = d;
            closest = SPEED_LEVELS[i];
        }
        i += 1;
    }
    if closest < 0.0 { 0.0 } else { closest }
}

/// Compute a checkpoint-limited target speed based on distance to the intersection center.
/// The closer to the center (0,0), the lower the allowed speed.
/// There are 10 radial bands mapping to the 10 SPEED_LEVELS.
#[inline]
pub fn checkpoint_speed_limit(distance_to_center: f32) -> f32 {
    // Radii thresholds (in world units). Outside => faster; inside => slower.
    // far (>= 450) => ~200 ... near (<= 50) => 40
    const RADII: [f32; 10] = [450.0, 400.0, 350.0, 300.0, 250.0, 200.0, 150.0, 120.0, 90.0, 50.0];

    // Find which band the car is in. If inside all radii, clamp to last (slowest) level.
    let mut idx = 0usize;
    while idx < RADII.len() && distance_to_center < RADII[idx] {
        idx += 1;
    }
    let level = if idx >= SPEED_LEVELS.len() {
        *SPEED_LEVELS.last().unwrap()
    } else {
        SPEED_LEVELS[idx]
    };
    quantize_speed(level)
}

pub struct VehiclePhysics;

impl VehiclePhysics {
    /// Safety distance increases with speed.
    pub fn calculate_safety_distance(speed: f32) -> f32 {
        MIN_SAFETY_DISTANCE + (speed * 0.5)
    }

    /// Integrate vehicle motion with discrete speeds + checkpoint-based caps.
    pub fn update_vehicle_physics(vehicle: &mut Vehicle, delta_time: f32) {
        // 1) Apply checkpoint limit based on distance to intersection center
        let dist = vehicle.position.magnitude();
        let checkpoint_limit = checkpoint_speed_limit(dist);
        if vehicle.target_speed > checkpoint_limit {
            vehicle.target_speed = checkpoint_limit;
        }

        // 2) Quantize current and target speeds to the nearest allowed levels
        vehicle.speed = quantize_speed(vehicle.speed);
        vehicle.target_speed = quantize_speed(vehicle.target_speed);

        // 3) Calculate acceleration towards target speed using discrete accel/decel
        let speed_diff = vehicle.target_speed - vehicle.speed;
        let acceleration = if speed_diff > 0.0 {
            MAX_ACCELERATION
        } else if speed_diff < 0.0 {
            -MAX_DECELERATION
        } else {
            0.0
        };

        // 4) Integrate speed with time step and clamp between 0 and target
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

        // 5) Move along the direction vector
        let movement = vehicle.direction.movement_vector() * vehicle.speed * delta_time;
        vehicle.position += movement;
    }

    /// Check if two vehicles are at a safe distance given their speeds.
    pub fn check_safe_distance(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let distance = (vehicle1.position - vehicle2.position).magnitude();
        let required_distance = Self::calculate_safety_distance(vehicle1.speed.max(vehicle2.speed));
        distance >= required_distance
    }
}
