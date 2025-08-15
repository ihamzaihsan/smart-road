use bevy::prelude::*;
use crate::components::Vehicle;
use crate::resources::RoadConfig;

pub fn load_vehicle_assets() {
    // stub: load textures / sprite sheets from assets/
}

pub fn spawn_vehicles(mut commands: Commands) {
    // stubbed spawn logic
    commands.spawn_empty().insert(Vehicle {
        id: 0,
        speed: 0.0,
        state: crate::components::VehicleState::Waiting,
        route: crate::components::Route::Straight,
    });
}

pub fn move_vehicles(mut query: Query<(&mut Transform, &mut Vehicle)>, _time: Res<Time>) {
    for (mut _transform, mut vehicle) in query.iter_mut() {
        // minimal stub movement
        if vehicle.state == crate::components::VehicleState::Moving {
            vehicle.speed += 0.1;
        }
    }
}
