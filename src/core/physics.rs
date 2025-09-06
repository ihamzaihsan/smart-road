use crate::components::Vehicle;
use nalgebra::Vector2;
use std::collections::HashMap;

pub const MIN_SAFETY_DISTANCE: f32 = 50.0;
pub const MAX_ACCELERATION: f32 = 700.0;
pub const MAX_DECELERATION: f32 = 1200.0; // Increased for harder braking

/// Defines twelve discrete speed levels for finer control, from fastest to slowest.
pub const SPEED_LEVELS: [f32; 12] = [
    600.0, 540.0, 480.0, 420.0, 360.0, 300.0, 
    260.0, 220.0, 180.0, 140.0, 100.0, 60.0 
];

/// Time headway constant for following distance (in seconds)
pub const TIME_HEADWAY: f32 = 1.1;
/// Minimum buffer distance regardless of speed
pub const MIN_BUFFER_DISTANCE: f32 = 80.0;

/// Hysteresis band as a percentage of current speed to prevent flapping
pub const HYSTERESIS_BAND: f32 = 0.08;

/// Conflict reservation window duration (seconds)
pub const CONFLICT_RESERVATION_WINDOW: f32 = 0.45;

/// Simple conflict calendar to manage intersection reservations
#[derive(Debug)]
pub struct ConflictCalendar {
    /// Maps snapped waypoint positions to (vehicle_id, end_time)
    reservations: HashMap<(i32, i32), (u64, f32)>,
}

impl ConflictCalendar {
    pub fn new() -> Self {
        Self {
            reservations: HashMap::new(),
        }
    }

    /// Snaps a position to a grid cell for conflict detection
    fn snap_position(pos: Vector2<f32>) -> (i32, i32) {
        const CELL_SIZE: f32 = 20.0;
        ((pos.x / CELL_SIZE).round() as i32, (pos.y / CELL_SIZE).round() as i32)
    }

    /// Attempts to reserve a conflict point for a vehicle
    pub fn try_reserve(&mut self, vehicle_id: u64, position: Vector2<f32>, start_time: f32, current_time: f32) -> Result<(), f32> {
        let key = Self::snap_position(position);
        let end_time = start_time + CONFLICT_RESERVATION_WINDOW;

        // Clean up expired reservations
        self.reservations.retain(|_, (_, expiry)| *expiry > current_time);

        if let Some((existing_id, existing_end)) = self.reservations.get(&key) {
            if *existing_id != vehicle_id {
                // Conflict detected - return the time gap
                return Err((start_time - *existing_end).abs());
            }
        }

        // Reserve the slot
        self.reservations.insert(key, (vehicle_id, end_time));
        Ok(())
    }

    /// Clears all reservations (useful for testing)
    pub fn clear(&mut self) {
        self.reservations.clear();
    }
}

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

/// Quantizes speed with hysteresis to prevent flapping near boundaries.
#[inline]
pub fn quantize_with_hysteresis(current_speed: f32, desired_speed: f32) -> f32 {
    let band = current_speed * HYSTERESIS_BAND;
    
    // If desired is very close to current, keep current to prevent jittering
    if (desired_speed - current_speed).abs() <= band {
        return current_speed;
    }
    
    quantize_speed(desired_speed)
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

/// Maps an ETA gap to a speed cap for yielding vehicles
#[inline]
pub fn eta_cap_from_gap(delta_t: f32) -> f32 {
    if delta_t < 0.4 {
        SPEED_LEVELS[11] // 60 - emergency slow
    } else if delta_t < 0.8 {
        SPEED_LEVELS[9] // 140 - very slow
    } else if delta_t < 1.2 {
        SPEED_LEVELS[7] // 220 - slow
    } else if delta_t < 1.8 {
        SPEED_LEVELS[5] // 300 - medium
    } else {
        SPEED_LEVELS[3] // 420 - cruise
    }
}

pub struct VehiclePhysics;

impl VehiclePhysics {
    /// Calculates the required safety distance based on the vehicle's speed.
    pub fn calculate_safety_distance(speed: f32) -> f32 {
        // Time-headway model: distance = speed * time_headway + buffer
        speed * TIME_HEADWAY + MIN_BUFFER_DISTANCE
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

        // Move vehicle along its fixed direction vector, ensuring no deviation.
        let movement_vector = vehicle.direction.movement_vector();
        vehicle.position += movement_vector * vehicle.speed * delta_time;

        // Advance waypoint if passed. This is crucial for speed control logic in game.rs.
        if let Some(waypoint) = vehicle.get_current_waypoint() {
            let to_waypoint = waypoint - vehicle.position;
            // If the dot product is negative, the waypoint is "behind" the vehicle relative to its direction of travel.
            if to_waypoint.dot(&movement_vector) < 0.0 {
                vehicle.advance_waypoint();
            }
        }
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