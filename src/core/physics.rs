use crate::components::Vehicle;

pub const MIN_SAFETY_DISTANCE: f32 = 60.0; 
pub const MAX_ACCELERATION: f32 = 200.0;  
pub const MAX_DECELERATION: f32 = 80.0;  

pub struct VehiclePhysics;

impl VehiclePhysics {
    
    pub fn calculate_safety_distance(speed: f32) -> f32 {
        // Safety distance increases with speed
        MIN_SAFETY_DISTANCE + (speed * 0.5)
    }

    pub fn update_vehicle_physics(vehicle: &mut Vehicle, delta_time: f32) {
        // Calculate acceleration towards target speed
        let speed_diff = vehicle.target_speed - vehicle.speed;
        let acceleration = if speed_diff > 0.0 {
            MAX_ACCELERATION
        } else {
            -MAX_DECELERATION
        };
        
        // Update speed
        vehicle.speed += acceleration * delta_time * speed_diff.signum();
        vehicle.speed = vehicle.speed.clamp(0.0, vehicle.target_speed);
        
        // Update position based on velocity
        let movement = vehicle.direction.movement_vector() * vehicle.speed * delta_time;
        vehicle.position += movement;
    }

    pub fn check_safe_distance(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let distance = (vehicle1.position - vehicle2.position).magnitude();
        let required_distance = Self::calculate_safety_distance(vehicle1.speed.max(vehicle2.speed));
        distance >= required_distance
    }
}