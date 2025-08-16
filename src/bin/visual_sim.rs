use bevy::prelude::*;
use smart_road::resources::{RoadConfig, SpawnManager, load_vehicle_assets};
use smart_road::systems::{
    roads::setup_roads,
    vehicles::{handle_keyboard_spawning, move_vehicles},
    cleanup::cleanup_finished_vehicles,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(RoadConfig::default())
        .insert_resource(SpawnManager::default())
        .add_systems(Startup, (setup_camera, setup_roads, load_vehicle_assets))
        .add_systems(Update, (
            handle_keyboard_spawning,
            move_vehicles,
            cleanup_finished_vehicles,
        ))
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

