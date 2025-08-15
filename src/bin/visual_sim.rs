use bevy::prelude::*;
use smart_road::resources::RoadConfig;
use smart_road::systems::roads::setup_roads;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(RoadConfig::default())
        .add_systems(Startup, (setup_camera, setup_roads))
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

