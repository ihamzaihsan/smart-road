use bevy::prelude::*;
use crate::systems::vehicles;

#[derive(Debug, Default, Resource)]
pub struct IdCounter(pub u64);

pub struct VehiclePlugin;

impl Plugin for VehiclePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(IdCounter::default())
            .add_system(vehicles::spawn_vehicles)
            .add_system(vehicles::move_vehicles);
    }
}
