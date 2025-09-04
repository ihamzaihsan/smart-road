use crate::components::Vehicle;
use nalgebra::Vector2;

pub const MIN_SAFETY_DISTANCE: f32 = 50.0;
pub const MAX_ACCELERATION: f32 = 700.0;
pub const MAX_DECELERATION: f32 = 100.0;

/// Defines seven discrete speed levels, from fastest to slowest.
pub const SPEED_LEVELS: [f32; 7] = [700.0, 600.0, 500.0, 400.0, 300.0, 200.0, 100.0];

/// Snaps a given speed to the nearest discrete speed level.
#[inline]
pub fn quantize_speed(speed: f32) -> f32 {
    SPEED_LEVELS.iter().fold(SPEED_LEVELS[0], |closest, &level| {
        if (speed - level).abs() < (speed - closest).abs() {
            level
        } else {
            closest
        }
    })
}

/// Determines a speed tier based on the vehicle's distance from a reference point.
#[inline]
pub fn speed_tier_from_distance(dist: f32, road_half_width: f32) -> f32 {
    const DISTANCE_MULTIPLIERS: [f32; 6] = [3.0, 2.5, 2.0, 1.5, 1.0, 0.5];

    if dist >= DISTANCE_MULTIPLIERS[0] * road_half_width {
        SPEED_LEVELS[0]
    } else if dist >= DISTANCE_MULTIPLIERS[1] * road_half_width {
        SPEED_LEVELS[1]
    } else if dist >= DISTANCE_MULTIPLIERS[2] * road_half_width {
        SPEED_LEVELS[2]
    } else if dist >= DISTANCE_MULTIPLIERS[3] * road_half_width {
        SPEED_LEVELS[3]
    } else if dist >= DISTANCE_MULTIPLIERS[4] * road_half_width {
        SPEED_LEVELS[4]
    } else if dist >= DISTANCE_MULTIPLIERS[5] * road_half_width {
        SPEED_LEVELS[5]
    } else {
        SPEED_LEVELS[6]
    }
}

pub struct VehiclePhysics;

impl VehiclePhysics {
    /// Calculates the required safety distance based on the vehicle's speed.
    pub fn calculate_safety_distance(speed: f32) -> f32 {
        // Increased safety distance slightly at higher speeds
        let multiplier = if speed > SPEED_LEVELS[2] { 1.0 } else { 0.8 }; // 440.0
        MIN_SAFETY_DISTANCE + (speed * multiplier)
    }

    /// Updates the vehicle's physics based on its target speed and environmental factors.
    pub fn update_vehicle_physics(vehicle: &mut Vehicle, delta_time: f32) {
        let speed_diff = vehicle.target_speed - vehicle.speed;
        
        let acceleration = if speed_diff > 0.0 {
            Self::get_max_acceleration(vehicle.speed)
        } else if speed_diff < 0.0 {
            -Self::get_max_deceleration(vehicle.speed)
        } else {
            0.0
        };

        vehicle.speed += acceleration * delta_time;
        vehicle.speed = vehicle.speed.clamp(0.0, vehicle.target_speed);

        let movement = vehicle.direction.movement_vector() * vehicle.speed * delta_time;
        vehicle.position += movement;
    }

    /// Determines the maximum acceleration for a vehicle based on its current speed.
    fn get_max_acceleration(speed: f32) -> f32 {
        if speed < SPEED_LEVELS[4] { // 280.0
            MAX_ACCELERATION
        } else if speed < SPEED_LEVELS[1] { // 520.0
            MAX_ACCELERATION * 0.85
        } else {
            MAX_ACCELERATION * 0.7
        }
    }

    /// Determines the maximum deceleration for a vehicle based on its current speed.
    fn get_max_deceleration(speed: f32) -> f32 {
        if speed > SPEED_LEVELS[2] { // 440.0
            MAX_DECELERATION * 1.5
        } else {
            MAX_DECELERATION
        }
    }

    /// Checks if two vehicles are maintaining a safe distance from each other.
    pub fn check_safe_distance(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let distance = (vehicle1.position - vehicle2.position).magnitude();
        let required_distance = Self::calculate_safety_distance(vehicle1.speed.max(vehicle2.speed));
        distance >= required_distance
    }

    /// Computes the estimated time for a vehicle to reach a specific point.
    pub fn time_to_point(vehicle: &Vehicle, point: Vector2<f32>) -> f32 {
        let dist = (point - vehicle.position).magnitude();
        let v = if vehicle.speed > 1.0 { vehicle.speed } else { vehicle.target_speed.max(1.0) };
        dist / v
    }
}