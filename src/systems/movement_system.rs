use crate::components::{Vehicle, VehicleState};
use crate::core::VehiclePhysics;
use crate::algorithms::SafetyManager;

pub struct MovementSystem;

impl MovementSystem {
    pub fn update_vehicles(vehicles: &mut [Vehicle], delta_time: f32) {
        // Apply safety adjustments before updating physics
        for i in 0..vehicles.len() {
            for j in 0..vehicles.len() {
                if i == j {
                    continue;
                }

                let (vehicle1, vehicle2) = if i < j {
                    let (left, right) = vehicles.split_at_mut(j);
                    (&mut left[i], &mut right[0])
                } else {
                    let (left, right) = vehicles.split_at_mut(i);
                    (&mut right[0], &mut left[j])
                };

                if vehicle1.direction == vehicle2.direction {
                    let (front_vehicle, rear_vehicle) = 
                        if (vehicle1.position - vehicle2.position).dot(&vehicle1.direction.movement_vector()) > 0.0 {
                            (vehicle1, vehicle2)
                        } else {
                            (vehicle2, vehicle1)
                        };

                    if let Some(safe_speed) = SafetyManager::calculate_safe_speed(rear_vehicle, front_vehicle) {
                        if rear_vehicle.target_speed > safe_speed {
                            rear_vehicle.target_speed = safe_speed;
                        }
                    }
                }
            }
        }

        // Update vehicle physics after safety adjustments
        for vehicle in vehicles.iter_mut() {
            if vehicle.state == VehicleState::Moving {
                Self::update_single_vehicle(vehicle, delta_time);
            }
        }
    }

    fn update_single_vehicle(vehicle: &mut Vehicle, delta_time: f32) {
        VehiclePhysics::update_vehicle_physics(vehicle, delta_time);
    }

    pub fn set_target_speed(vehicle: &mut Vehicle, target_speed: f32) {
        vehicle.target_speed = target_speed.max(0.0);
    }

    pub fn stop_vehicle(vehicle: &mut Vehicle) {
        vehicle.speed = 0.0;
        vehicle.target_speed = 0.0;
        vehicle.state = VehicleState::Stopped;
    }
}