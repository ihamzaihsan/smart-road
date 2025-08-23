use bevy::prelude::*;
use crate::components::{Vehicle, VehicleState};

pub fn cleanup_finished_vehicles(
    mut commands: Commands,
    query: Query<(Entity, &Vehicle), With<Vehicle>>,
) {
    for (entity, vehicle) in query.iter() {
        if vehicle.state == VehicleState::Stopped {
            commands.entity(entity).despawn();
        }
    }
}

pub fn cleanup_finished(_commands: Commands) {
    // This function is now replaced by cleanup_finished_vehicles
    // Kept as stub for existing plugin compatibility
}