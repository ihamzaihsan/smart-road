use crate::components::Vehicle;

pub struct CleanupSystem;

impl CleanupSystem {
    
    pub fn cleanup_vehicles(vehicles: &mut Vec<Vehicle>, cleanup_distance: f32) {
        vehicles.retain(|vehicle| {
            let distance_from_center = vehicle.position.magnitude();
            distance_from_center <= cleanup_distance
        });
    }

    pub fn count_vehicles_to_cleanup(vehicles: &[Vehicle], cleanup_distance: f32) -> usize {
        vehicles.iter()
            .filter(|vehicle| {
                let distance_from_center = vehicle.position.magnitude();
                distance_from_center > cleanup_distance
            })
            .count()
    }
}