use crate::components::{Route, Vehicle, VehicleState};
use crate::core::waypoints::Track;
use crate::resources::{center_separation, CROSSING_SPEED, CRUISE_SPEED, TURN_SPEED};

/// Calculate every control decision from one snapshot, then integrate movement.
pub fn update_vehicles(vehicles: &mut [Vehicle], tracks: &[Track], dt: f32) {
    let mut limits = Vec::with_capacity(vehicles.len());
    for vehicle in vehicles.iter() {
        let track = &tracks[vehicle.track_index];
        let mut available = track.length - vehicle.progress;
        if !vehicle.admitted {
            available = available.min((track.stop_progress - vehicle.progress).max(0.0));
        }
        // Scalar progress identifies a leader even through a curve.
        for other in vehicles.iter() {
            if other.track_index == vehicle.track_index && other.progress > vehicle.progress {
                let gap = other.progress - vehicle.progress - center_separation();
                available = available.min(gap.max(0.0));
            }
        }
        let before_turn = vehicle.progress < track.clear_progress
            && track.stop_progress - vehicle.progress < 100.0;
        let free_speed = if before_turn {
            if track.route == Route::Straight {
                CROSSING_SPEED
            } else {
                TURN_SPEED
            }
        } else {
            CRUISE_SPEED
        };
        // Leave one integration step of braking margin before a stop/leader.
        let braking_distance =
            (available - vehicle.speed * dt - 0.5 * vehicle.deceleration * dt * dt).max(0.0);
        let safe_speed = (2.0 * vehicle.deceleration * braking_distance).sqrt();
        limits.push((free_speed.min(safe_speed), available));
    }

    for (vehicle, (target, available)) in vehicles.iter_mut().zip(limits) {
        let previous_speed = vehicle.speed;
        let rate = if target >= previous_speed {
            vehicle.acceleration
        } else {
            vehicle.deceleration
        };
        let difference = (target - previous_speed).clamp(-rate * dt, rate * dt);
        vehicle.speed = (previous_speed + difference).max(0.0);
        let step = ((previous_speed + vehicle.speed) * 0.5 * dt).min(available);
        vehicle.progress += step;
        // The distance cap protects a geometric limit without resetting speed.
        // Both acceleration and braking stay bounded, including a zero target.
        let track = &tracks[vehicle.track_index];
        (vehicle.position, vehicle.rotation) = track.pose(vehicle.progress);
        vehicle.state = if vehicle.progress >= track.clear_progress {
            VehicleState::Exiting
        } else if vehicle.admitted {
            VehicleState::Crossing
        } else if vehicle.speed < 1.0 {
            VehicleState::Waiting
        } else {
            VehicleState::Approaching
        };
        if vehicle.state == VehicleState::Waiting {
            vehicle.waiting_time += dt as f64;
        }
    }
}
