use bevy::prelude::*;
use crate::systems::roads;

pub struct RoadPlugin;

impl Plugin for RoadPlugin {
    fn build(&self, app: &mut App) {
        app.add_startup_system(roads::setup_roads);
    }
}
