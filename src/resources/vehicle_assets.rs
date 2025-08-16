use bevy::prelude::*;
use rand::Rng;

/// Resource that holds loaded vehicle sprite textures
#[derive(Resource, Debug)]
pub struct VehicleAssets {
    pub car_textures: Vec<Handle<Image>>,
}

impl VehicleAssets {
    /// Create a new VehicleAssets resource with loaded textures
    pub fn new(asset_server: &AssetServer) -> Self {
        let car_textures = vec![
            asset_server.load("Car.png"),
            asset_server.load("Audi.png"),
            asset_server.load("Mini_van.png"),
            asset_server.load("taxi.png"),
            asset_server.load("Police.png"),
        ];

        Self { car_textures }
    }

    /// Get a random car texture
    pub fn get_random_texture(&self) -> Handle<Image> {
        let mut rng = rand::thread_rng();
        let index = rng.gen_range(0..self.car_textures.len());
        self.car_textures[index].clone()
    }
}

/// System to load vehicle assets during startup
pub fn load_vehicle_assets(mut commands: Commands, asset_server: Res<AssetServer>) {
    let vehicle_assets = VehicleAssets::new(&asset_server);
    commands.insert_resource(vehicle_assets);
}
