use rand::Rng;
use sdl2::render::Texture;

/// Holds loaded vehicle sprite textures for SDL2
pub struct VehicleAssets<'a> {
    pub car_textures: Vec<Texture<'a>>,
    pub texture_names: Vec<String>,
}

impl<'a> VehicleAssets<'a> {
    /// Create a new VehicleAssets with empty textures (to be loaded later)
    pub fn new() -> Self {
        Self {
            car_textures: Vec::new(),
            texture_names: vec![
                "Car.png".to_string(),
                "Audi.png".to_string(),
                "Mini_van.png".to_string(),
                "taxi.png".to_string(),
                "Police.png".to_string(),
            ],
        }
    }

    /// Get a random car texture index
    pub fn get_random_texture_index(&self) -> usize {
        let mut rng = rand::thread_rng();
        rng.gen_range(0..self.car_textures.len().max(1))
    }

    /// Get the number of available textures
    pub fn texture_count(&self) -> usize {
        self.car_textures.len()
    }
}
