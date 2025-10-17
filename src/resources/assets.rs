use crate::resources::{STYLE_COUNT, VEHICLE_LENGTH, VEHICLE_WIDTH};
use sdl2::{
    pixels::{Color, PixelFormatEnum},
    rect::Rect,
    render::{BlendMode, Texture, TextureCreator},
    surface::Surface,
    video::WindowContext,
};

/// Original top-down vehicle artwork, generated once and cached as SDL textures.
/// Rectangles form the body, wheels, windows, lights and a distinct roof stripe.
/// No external images, image decoder, font package or third-party artwork is used.
pub struct VehicleAssets<'a> {
    pub textures: Vec<Texture<'a>>,
}

impl<'a> VehicleAssets<'a> {
    pub fn new(creator: &'a TextureCreator<WindowContext>) -> Result<Self, String> {
        let palette: [Color; STYLE_COUNT] = [
            Color::RGB(73, 192, 178),
            Color::RGB(243, 177, 75),
            Color::RGB(123, 151, 235),
            Color::RGB(225, 119, 138),
            Color::RGB(199, 208, 218),
        ];
        let mut textures = Vec::with_capacity(STYLE_COUNT);
        for color in palette {
            let mut surface = Surface::new(
                VEHICLE_WIDTH as u32,
                VEHICLE_LENGTH as u32,
                PixelFormatEnum::RGBA32,
            )?;
            surface.fill_rect(None, Color::RGBA(0, 0, 0, 0))?;
            for y in [5, 23] {
                surface.fill_rect(Rect::new(0, y, 3, 5), Color::RGB(17, 24, 32))?;
                surface.fill_rect(Rect::new(13, y, 3, 5), Color::RGB(17, 24, 32))?;
            }
            surface.fill_rect(Rect::new(2, 1, 12, 30), color)?;
            surface.fill_rect(Rect::new(3, 8, 10, 6), Color::RGB(31, 51, 64))?;
            surface.fill_rect(Rect::new(3, 21, 10, 5), Color::RGB(31, 51, 64))?;
            surface.fill_rect(Rect::new(7, 14, 2, 7), Color::RGB(237, 244, 240))?;
            for x in [3, 11] {
                surface.fill_rect(Rect::new(x, 1, 2, 3), Color::RGB(255, 239, 175))?;
                surface.fill_rect(Rect::new(x, 28, 2, 2), Color::RGB(198, 67, 78))?;
            }
            let mut texture = creator
                .create_texture_from_surface(&surface)
                .map_err(|e| e.to_string())?;
            texture.set_blend_mode(BlendMode::Blend);
            textures.push(texture);
        }
        Ok(Self { textures })
    }
}
