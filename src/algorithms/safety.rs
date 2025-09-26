use crate::components::Vehicle;
use crate::core::VehiclePhysics;

pub struct SafetyManager;

impl SafetyManager {
    pub fn calculate_safe_speed(vehicle: &Vehicle, other_vehicle: &Vehicle) -> Option<f32> {
        let distance = (vehicle.position - other_vehicle.position).magnitude();
        let required_distance = VehiclePhysics::calculate_safety_distance(vehicle.speed);

        if distance < required_distance {
            let safe_speed = other_vehicle.speed * (distance / required_distance);
            Some(safe_speed.max(0.0))
        } else {
            None
        }
    }
}
