use crate::components::Vehicle;
use crate::core::VehiclePhysics;

pub struct CollisionSystem;

impl CollisionSystem {
    
    pub fn check_and_prevent_collisions(vehicles: &mut Vec<Vehicle>) {
        let vehicle_count = vehicles.len();
        
        for i in 0..vehicle_count {
            for j in (i + 1)..vehicle_count {
                let (vehicle1, vehicle2) = Self::get_two_vehicles_mut(vehicles, i, j);
                
                if !VehiclePhysics::check_safe_distance(vehicle1, vehicle2) {
                    Self::handle_collision_avoidance(vehicle1, vehicle2);
                }
            }
        }
    }

    fn get_two_vehicles_mut(vehicles: &mut Vec<Vehicle>, i: usize, j: usize) -> (&mut Vehicle, &mut Vehicle) {
        if i < j {
            let (left, right) = vehicles.split_at_mut(j);
            (&mut left[i], &mut right[0])
        } else {
            let (left, right) = vehicles.split_at_mut(i);
            (&mut right[0], &mut left[j])
        }
    }

    fn handle_collision_avoidance(vehicle1: &mut Vehicle, vehicle2: &mut Vehicle) {
        // Simple collision avoidance: slow down the faster vehicle
        if vehicle1.speed > vehicle2.speed {
            vehicle1.target_speed = vehicle1.target_speed * 0.7;
        } else {
            vehicle2.target_speed = vehicle2.target_speed * 0.7;
        }
    }

    pub fn count_close_calls(vehicles: &[Vehicle]) -> u32 {
        let mut close_calls = 0;
        
        for i in 0..vehicles.len() {
            for j in (i + 1)..vehicles.len() {
                if !VehiclePhysics::check_safe_distance(&vehicles[i], &vehicles[j]) {
                    close_calls += 1;
                }
            }
        }
        
        close_calls
    }
}