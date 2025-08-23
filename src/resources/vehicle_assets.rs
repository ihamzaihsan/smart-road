use rand::Rng;
use sdl2::render::Texture;

pub struct VehicleAssets<'a> {
    pub car_textures: Vec<Texture<'a>>,
    pub texture_names: Vec<String>,
}

impl<'a> VehicleAssets<'a> {
    
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

    pub fn get_random_texture_index(&self) -> usize {
        let mut rng = rand::thread_rng();
        rng.gen_range(0..self.car_textures.len().max(1))
    }

    pub fn texture_count(&self) -> usize {
        self.car_textures.len()
    }
}