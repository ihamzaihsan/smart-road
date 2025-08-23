use crate::components::Vehicle;
use crate::core::IntersectionManager;

pub struct IntersectionSystem {
    manager: IntersectionManager,
}

impl IntersectionSystem {
    pub fn new() -> Self {
        Self {
            manager: IntersectionManager::new(),
        }
    }

    pub fn update(&mut self, _vehicles: &mut Vec<Vehicle>) {
        self.manager.schedule_vehicles();
        
        // TODO: Implement smart intersection logic
        // - Detect vehicles approaching intersection
        // - Calculate optimal speeds and timing
        // - Manage vehicle priorities
        // - Prevent conflicts
    }

    pub fn is_in_intersection(vehicle: &Vehicle, intersection_size: f32) -> bool {
        let half_size = intersection_size / 2.0;
        vehicle.position.x.abs() <= half_size && vehicle.position.y.abs() <= half_size
    }

    pub fn is_approaching_intersection(vehicle: &Vehicle, detection_distance: f32) -> bool {
        let distance_to_center = vehicle.position.magnitude();
        distance_to_center <= detection_distance
    }
}