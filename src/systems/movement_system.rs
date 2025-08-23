use crate::components::{Vehicle, VehicleState};
use crate::core::VehiclePhysics;

pub struct MovementSystem;

impl MovementSystem {
    
    pub fn update_vehicles(vehicles: &mut Vec<Vehicle>, delta_time: f32) {
        for vehicle in vehicles.iter_mut() {
            if vehicle.state != VehicleState::Moving {
                continue;
            }
            
            Self::update_single_vehicle(vehicle, delta_time);
        }
    }

    fn update_single_vehicle(vehicle: &mut Vehicle, delta_time: f32) {
        // Use physics system for movement calculations
        VehiclePhysics::update_vehicle_physics(vehicle, delta_time);
    }

    pub fn set_target_speed(vehicle: &mut Vehicle, target_speed: f32) {
        vehicle.target_speed = target_speed.max(0.0); // Ensure non-negative speed
    }

    pub fn stop_vehicle(vehicle: &mut Vehicle) {
        vehicle.speed = 0.0;
        vehicle.target_speed = 0.0;
        vehicle.state = VehicleState::Stopped;
    }
}