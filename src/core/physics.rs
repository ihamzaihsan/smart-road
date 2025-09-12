use crate::components::Vehicle;
use nalgebra::Vector2;
use std::collections::HashMap;
pub const MIN_SAFETY_DISTANCE: f32 = 100.0;
pub const MAX_ACCELERATION: f32 = 3000.0;
pub const MAX_DECELERATION: f32 = 2000.0;
pub const EMERGENCY_DECELERATION: f32 = 3000.0;
pub const SPEED_LEVELS: [f32; 12] = [
    3000.0, 2500.0, 2000.0, 1800.0, 1600.0, 1400.0, 1200.0, 1000.0, 600.0, 400.0, 200.0, 100.0,
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
        ((pos.x / CELL_SIZE).round() as i32, (pos.y / CELL_SIZE).round() as i32)
    }
    pub fn try_reserve(&mut self, vehicle_id: u64, position: Vector2<f32>, start_time: f32, current_time: f32) -> Result<(), f32> {
        let key = Self::snap_position(position);
        let end_time = start_time + CONFLICT_RESERVATION_WINDOW;
        self.reservations.retain(|_, (_, expiry)| *expiry > current_time);
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
    match SPEED_LEVELS.binary_search_by(|level| level.partial_cmp(&speed).unwrap().reverse()) {
        Ok(index) => SPEED_LEVELS[index], 
        Err(index) => {
            if index == 0 {
                SPEED_LEVELS[0]
            } else if index == SPEED_LEVELS.len() {
                SPEED_LEVELS[SPEED_LEVELS.len() - 1]
            } else {
                let upper_bound = SPEED_LEVELS[index - 1];
                let lower_bound = SPEED_LEVELS[index];
                if (upper_bound - speed) <= (speed - lower_bound) {
                    upper_bound
                } else {
                    lower_bound
                }
            }
        }
    }
}

#[inline]
pub fn quantize_with_hysteresis(current_speed: f32, desired_speed: f32) -> f32 {
    let band = current_speed * (HYSTERESIS_BAND + 0.03);
    if (desired_speed - current_speed).abs() <= band {
        return current_speed;
    }
    quantize_speed(desired_speed)
}

#[inline]
pub fn speed_tier_from_distance(dist: f32, road_half_width: f32) -> f32 {
    const DISTANCE_TIERS: &[(f32, usize)] = &[
        (3.5, 0),  
        (3.0, 1),
        (2.75, 2),
        (2.5, 3),
        (2.25, 4),
        (1.75, 5),
        (1.25, 6),
    ];

    for &(multiplier, speed_index) in DISTANCE_TIERS {
        if dist >= multiplier * road_half_width {
            return SPEED_LEVELS[speed_index];
        }
    }
    
    SPEED_LEVELS[8]
}
#[inline]
pub fn eta_cap_from_gap(delta_t: f32) -> f32 {
    const GAP_TIERS: &[(f32, usize)] = &[
        (0.25, 11), 
        (0.75, 10),
        (1.0, 9),
        (1.25, 8),
        (1.5, 7),
        (2.0, 6),
    ];

    for &(gap, speed_index) in GAP_TIERS {
        if delta_t < gap {
            return SPEED_LEVELS[speed_index];
        }
    }
    
    SPEED_LEVELS[4] 
}
pub struct VehiclePhysics;
impl VehiclePhysics {
    pub fn calculate_safety_distance(speed: f32) -> f32 {
        let reaction_distance = speed * TIME_HEADWAY;
        let braking_distance = (speed * speed) / (2.0 * MAX_DECELERATION);
        (reaction_distance + braking_distance + MIN_BUFFER_DISTANCE).max(MIN_SAFETY_DISTANCE)
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
        vehicle.speed = vehicle.speed.clamp(0.0, vehicle.target_speed.max(vehicle.speed * 0.95));
        let movement_vector = vehicle.direction.movement_vector();
        vehicle.position += movement_vector * vehicle.speed * delta_time;
        if let Some(waypoint) = vehicle.get_current_waypoint() {
            let to_waypoint = waypoint - vehicle.position;
            if to_waypoint.dot(&movement_vector) < 0.0 {
                vehicle.advance_waypoint();
            }
        }
    }
    fn get_max_acceleration(speed: f32) -> f32 {
        const ACCEL_TIERS: &[(f32, f32)] = &[(0.0, 1.0), (SPEED_LEVELS[3], 0.6)];

        let (lower_speed, lower_factor) = ACCEL_TIERS[0];
        let (upper_speed, upper_factor) = ACCEL_TIERS[1];

        if speed <= lower_speed {
            return MAX_ACCELERATION * lower_factor;
        }
        if speed >= upper_speed {
            return MAX_ACCELERATION * upper_factor;
        }

        let factor = (speed - lower_speed) / (upper_speed - lower_speed);
        let interpolated_factor = lower_factor + factor * (upper_factor - lower_factor);
        
        MAX_ACCELERATION * interpolated_factor
    }

    fn get_max_deceleration(speed: f32) -> f32 {
        const DECEL_TIERS: &[(f32, f32)] = &[(SPEED_LEVELS[5], 1.0), (SPEED_LEVELS[2], 1.8)];

        let (lower_speed, lower_factor) = DECEL_TIERS[0];
        let (upper_speed, upper_factor) = DECEL_TIERS[1];

        if speed <= lower_speed {
            return MAX_DECELERATION * lower_factor;
        }
        if speed >= upper_speed {
            return MAX_DECELERATION * upper_factor;
        }

        let factor = (speed - lower_speed) / (upper_speed - lower_speed);
        let interpolated_factor = lower_factor + factor * (upper_factor - lower_factor);

        MAX_DECELERATION * interpolated_factor
    }
    pub fn check_safe_distance(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let distance = (vehicle1.position - vehicle2.position).magnitude();
        let relative_speed = vehicle1.speed - vehicle2.speed;
        
        let required_distance = if relative_speed > 0.0 {
            let time_to_close_gap = distance / relative_speed;
            let stopping_time = vehicle1.speed / MAX_DECELERATION;
            Self::calculate_safety_distance(vehicle1.speed) + (relative_speed * (time_to_close_gap - stopping_time)).max(0.0)
        } else {
            Self::calculate_safety_distance(vehicle1.speed)
        };

        distance >= required_distance
    }

    pub fn time_to_point(vehicle: &Vehicle, point: Vector2<f32>) -> f32 {
        const EPSILON: f32 = 1e-6;
        let dist = (point - vehicle.position).magnitude();
        
        let speed_at_point = (vehicle.speed.powi(2) + 2.0 * MAX_ACCELERATION * dist).sqrt();
        let average_speed = (vehicle.speed + speed_at_point) / 2.0;

        if average_speed < EPSILON {
            f32::MAX 
        } else {
            dist / average_speed
        }
    }
}
