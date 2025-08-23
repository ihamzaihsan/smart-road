use crate::components::Vehicle;

pub struct SafetyManager;

impl SafetyManager {
    
    pub fn calculate_safety_distance(speed: f32) -> f32 {
        // Safety distance increases with speed (simplified model)
        30.0 + (speed * 0.5)
    }

    pub fn check_safe_distance(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
        let distance = (vehicle1.position - vehicle2.position).magnitude();
        let required_distance = Self::calculate_safety_distance(vehicle1.speed.max(vehicle2.speed));
        distance >= required_distance
    }

    pub fn calculate_safe_speed(vehicle: &Vehicle, nearby_vehicles: &[Vehicle]) -> f32 {
        let mut safe_speed = vehicle.target_speed;
        
        for other in nearby_vehicles {
            if other.id == vehicle.id {
                continue;
            }
            
            let distance = (vehicle.position - other.position).magnitude();
            let required_distance = Self::calculate_safety_distance(vehicle.speed);
            
            if distance < required_distance * 1.5 {
                // Reduce speed if too close
                safe_speed = safe_speed.min(other.speed * 0.8);
            }
        }
        
        safe_speed.max(0.0)
    }
}