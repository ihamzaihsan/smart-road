use crate::components::{Direction, Vehicle};
use nalgebra::Vector2;
use std::collections::HashMap;
pub const MIN_SAFETY_DISTANCE: f32 = 75.0;
pub const MAX_ACCELERATION: f32 = 250.0; 
pub const MAX_DECELERATION: f32 = 2500.0; 
pub const EMERGENCY_DECELERATION: f32 = 4000.0; 
pub const SPEED_LEVELS: [f32; 5] = [
    300.0, 200.0, 160.0, 120.0, 80.0,
];
pub const TIME_HEADWAY: f32 = 2.0;
pub const MIN_BUFFER_DISTANCE: f32 = 120.0;
pub const CRITICAL_DISTANCE: f32 = 60.0;
pub const HYSTERESIS_BAND: f32 = 0.12;
pub const CONFLICT_RESERVATION_WINDOW: f32 = 0.8;
#[derive(Debug)]
pub struct ConflictCalendar {
    reservations: HashMap<(i32, i32), (u64, f32)>,
}
impl ConflictCalendar {
    pub fn new() -> Self {
        Self {
            reservations: HashMap::new(),
        }
    }
    fn snap_position(pos: Vector2<f32>) -> (i32, i32) {
        const CELL_SIZE: f32 = 20.0;
        (
            (pos.x / CELL_SIZE).round() as i32,
            (pos.y / CELL_SIZE).round() as i32,
        )
    }
    pub fn try_reserve(
        &mut self,
        vehicle_id: u64,
        position: Vector2<f32>,
        start_time: f32,
        current_time: f32,
    ) -> Result<(), f32> {
        let key = Self::snap_position(position);
        let end_time = start_time + CONFLICT_RESERVATION_WINDOW;
        self.reservations
            .retain(|_, (_, expiry)| *expiry > current_time);
        if let Some((existing_id, existing_end)) = self.reservations.get(&key) {
            if *existing_id != vehicle_id {
                return Err((start_time - *existing_end).abs());
            }
        }
        self.reservations.insert(key, (vehicle_id, end_time));
        Ok(())
    }

    pub fn clear(&mut self) {
        self.reservations.clear();
    }
}
#[inline]
pub fn quantize_speed(speed: f32) -> f32 {
    SPEED_LEVELS
        .iter()
        .fold(SPEED_LEVELS[0], |closest, &level| {
            if (speed - level).abs() < (speed - closest).abs() {
                level
            } else {
                closest
            }
        })
}
#[inline]
pub fn quantize_with_hysteresis(current_speed: f32, desired_speed: f32) -> f32 {
    let band = current_speed * HYSTERESIS_BAND;
    if (desired_speed - current_speed).abs() <= band {
        return current_speed;
    }
    quantize_speed(desired_speed)
}
#[inline]
pub fn speed_tier_from_distance(dist: f32, road_half_width: f32) -> f32 {
    // Make speed reduction less aggressive - only slow down when very close to waypoint
    const DISTANCE_MULTIPLIERS: [f32; 4] = [1.0, 0.5, 0.25, 0.1];
    if dist >= DISTANCE_MULTIPLIERS[0] * road_half_width {
        SPEED_LEVELS[1] // Medium speed instead of slowest
    } else if dist >= DISTANCE_MULTIPLIERS[1] * road_half_width {
        SPEED_LEVELS[2] // Still reasonable speed
    } else if dist >= DISTANCE_MULTIPLIERS[2] * road_half_width {
        SPEED_LEVELS[3] // Slow down when approaching
    } else {
        SPEED_LEVELS[4] // Only slowest when very close
    }
}
#[inline]
pub fn eta_cap_from_gap(delta_t: f32, vehicle_a: &Vehicle, vehicle_b: &Vehicle) -> f32 {
    let speed_a = vehicle_a.speed;
    let speed_b = vehicle_b.speed;
    let distance = (vehicle_a.position - vehicle_b.position).magnitude();

    // Calculate the time it would take for vehicle_a to reach vehicle_b
    let time_to_collision = if (speed_a + 1e-5) > 0.0 {
        (distance - vehicle_a.width / 2.0) / (speed_a + 1e-5)
    } else {
        f32::INFINITY
    };

    // Use current speed for estimation since acceleration is not available
    let speed_at_collision = speed_a;

    // Determine the speed level based on the speed at the time of collision
    if speed_at_collision < SPEED_LEVELS[4] {
        SPEED_LEVELS[4] // Slowest (80 units/sec)
    } else if speed_at_collision < SPEED_LEVELS[3] {
        SPEED_LEVELS[3] // Slow (120 units/sec)
    } else if speed_at_collision < SPEED_LEVELS[2] {
        SPEED_LEVELS[2] // Medium (160 units/sec)
    } else if speed_at_collision < SPEED_LEVELS[1] {
        SPEED_LEVELS[1] // Fast (200 units/sec)
    } else {
        SPEED_LEVELS[0] // Fastest (300 units/sec)
    }
}
pub struct VehiclePhysics;
impl VehiclePhysics {
  pub fn calculate_safety_distance(speed: f32) -> f32 {
    let time_headway = TIME_HEADWAY * 1.2; 
    (speed * time_headway + MIN_BUFFER_DISTANCE).max(MIN_SAFETY_DISTANCE) + 50.0 
}

    pub fn check_emergency_brake_needed(vehicle: &Vehicle, other: &Vehicle) -> bool {
        let distance = (vehicle.position - other.position).magnitude();
        let relative_speed = vehicle.speed - other.speed;
        if relative_speed > 0.0 {
            let time_to_collision = distance / relative_speed;
            time_to_collision < 1.0 && distance < CRITICAL_DISTANCE * 2.0
        } else {
            false
        }
    }
    pub fn detect_collision_risk(vehicle: &Vehicle, other_vehicles: &[Vehicle]) -> (bool, f32) {
        let mut min_safe_speed = vehicle.target_speed;
        let mut emergency_brake = false;
        for other in other_vehicles {
            if other.id == vehicle.id {
                continue;
            }
            let distance = (vehicle.position - other.position).magnitude();
            if Self::are_on_collision_course(vehicle, other) {
                if Self::check_emergency_brake_needed(vehicle, other) {
                    emergency_brake = true;
                    min_safe_speed = 0.0;
                    break;
                }
                let safe_distance = Self::calculate_safety_distance(vehicle.speed);
                if distance < safe_distance * 2.0 {
                    let safety_factor = (distance / safe_distance).clamp(0.1, 1.0);
                    let safe_speed = other.speed * safety_factor * 0.8;
                    min_safe_speed = min_safe_speed.min(safe_speed);
                }
            }
        }
        (emergency_brake, min_safe_speed)
    }
    pub fn are_on_collision_course(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let distance = (vehicle1.position - vehicle2.position).magnitude();
        if distance > 300.0 {
            return false;
        }
        let relative_position = vehicle2.position - vehicle1.position;
        let vehicle1_direction = vehicle1.direction.movement_vector();
        let vehicle2_direction = vehicle2.direction.movement_vector();
        if vehicle1.direction == vehicle2.direction {
            let to_other = relative_position.normalize();
            let alignment = vehicle1_direction.dot(&to_other);
            return alignment > 0.7;
        }
        let v1_future = vehicle1.position + vehicle1_direction * distance;
        let v2_future = vehicle2.position + vehicle2_direction * distance;
        let future_distance = (v1_future - v2_future).magnitude();
        future_distance < 80.0
    }
    pub fn update_vehicle_physics(vehicle: &mut Vehicle, delta_time: f32) {
        let speed_diff = vehicle.target_speed - vehicle.speed;
        let acceleration = if speed_diff > 0.0 {
            Self::get_max_acceleration(vehicle.speed)
        } else if speed_diff < 0.0 {
            if vehicle.target_speed < 30.0 && vehicle.speed > 100.0 {
                -EMERGENCY_DECELERATION
            } else {
                -Self::get_max_deceleration(vehicle.speed)
            }
        } else {
            0.0
        };
        vehicle.speed += acceleration * delta_time;
        vehicle.speed = vehicle
            .speed
            .clamp(0.0, vehicle.target_speed.max(vehicle.speed * 0.95));

        Self::update_position_along_path(vehicle, delta_time);
    }

    /// Update vehicle position without changing speed (for AlwaysMoving vehicles)
    pub fn update_vehicle_position_only(vehicle: &mut Vehicle, delta_time: f32) {
        // Skip speed changes, only update position based on current fixed speed
        Self::update_position_along_path(vehicle, delta_time);
    }

    /// Shared logic for updating vehicle position along its path
    fn update_position_along_path(vehicle: &mut Vehicle, delta_time: f32) {
        let mut remaining_distance = vehicle.speed * delta_time;
        while remaining_distance > 0.0 {
            if let Some(waypoint) = vehicle.get_current_waypoint() {
                let to_waypoint = waypoint - vehicle.position;
                let distance = to_waypoint.magnitude();
                if distance < 1.0 {
                    vehicle.advance_waypoint();
                    continue;
                }

                let direction_vec = to_waypoint / distance;
                Self::update_heading(vehicle, &direction_vec);

                if remaining_distance < distance {
                    vehicle.position += direction_vec * remaining_distance;
                    remaining_distance = 0.0;
                } else {
                    vehicle.position = waypoint;
                    vehicle.advance_waypoint();
                    remaining_distance -= distance;
                }
            } else {
                let movement_vector = vehicle.direction.movement_vector();
                if movement_vector.magnitude_squared() > 0.0 {
                    Self::update_heading(vehicle, &movement_vector);
                    vehicle.position += movement_vector * remaining_distance;
                }
                remaining_distance = 0.0;
            }
        }
    }

    fn update_heading(vehicle: &mut Vehicle, direction_vec: &Vector2<f32>) {
        let angle = std::f32::consts::FRAC_PI_2 - direction_vec.y.atan2(direction_vec.x);
        vehicle.rotation = angle;
        if let Some(new_direction) =
            Direction::from_vector(Vector2::new(direction_vec.x, direction_vec.y))
        {
            vehicle.direction = new_direction;
        }
    }
    fn get_max_acceleration(speed: f32) -> f32 {
        if speed < SPEED_LEVELS[3] {
            MAX_ACCELERATION
        } else if speed < SPEED_LEVELS[1] {
            MAX_ACCELERATION * 0.8
        } else {
            MAX_ACCELERATION * 0.6
        }
    }
    fn get_max_deceleration(speed: f32) -> f32 {
        if speed > SPEED_LEVELS[1] {
            MAX_DECELERATION * 1.8
        } else if speed > SPEED_LEVELS[2] {
            MAX_DECELERATION * 1.4
        } else {
            MAX_DECELERATION
        }
    }
    pub fn check_safe_distance(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let distance = (vehicle1.position - vehicle2.position).magnitude();
        let required_distance = Self::calculate_safety_distance(vehicle1.speed.max(vehicle2.speed));
        distance >= required_distance
    }
    pub fn time_to_point(vehicle: &Vehicle, point: Vector2<f32>) -> f32 {
        let dist = (point - vehicle.position).magnitude();
        let v = if vehicle.speed > 1.0 {
            vehicle.speed
        } else {
            vehicle.target_speed.max(1.0)
        };
        dist / v
    }
}
