use sdl2::render::{Canvas, TextureCreator};
use sdl2::video::{Window, WindowContext};
use sdl2::rect::Rect;
use sdl2::pixels::Color;
use crate::components::Vehicle;
use crate::resources::{RoadConfig, VehicleAssets};

/// SDL2 rendering system for the smart-road simulator
pub struct Renderer<'a> {
    pub canvas: Canvas<Window>,
    pub texture_creator: &'a TextureCreator<WindowContext>,
}

impl<'a> Renderer<'a> {
    /// Create a new renderer with the given canvas and texture creator
    pub fn new(canvas: Canvas<Window>, texture_creator: &'a TextureCreator<WindowContext>) -> Self {
        Self {
            canvas,
            texture_creator,
        }
    }

    /// Clear the screen with a background color
    pub fn clear(&mut self) {
        self.canvas.set_draw_color(Color::RGB(50, 50, 50)); // Dark gray background
        self.canvas.clear();
    }

    /// Present the rendered frame
    pub fn present(&mut self) {
        self.canvas.present();
    }

    /// Render the road intersection
    pub fn render_roads(&mut self, road_config: &RoadConfig) -> Result<(), String> {
        let (window_width, window_height) = self.canvas.output_size().map_err(|e| e.to_string())?;
        let center_x = window_width as i32 / 2;
        let center_y = window_height as i32 / 2;

        // Calculate road width from lane configuration
        let road_width = road_config.lane_width * road_config.num_lanes as f32 * 2.0; // 2 directions

        // Road surface color
        self.canvas.set_draw_color(Color::RGB(80, 80, 80)); // Dark gray for roads

        // Draw horizontal road
        let road_rect = Rect::new(
            0,
            center_y - (road_width / 2.0) as i32,
            window_width,
            road_width as u32,
        );
        self.canvas.fill_rect(road_rect).map_err(|e| e.to_string())?;

        // Draw vertical road
        let road_rect = Rect::new(
            center_x - (road_width / 2.0) as i32,
            0,
            road_width as u32,
            window_height,
        );
        self.canvas.fill_rect(road_rect).map_err(|e| e.to_string())?;

        // Draw lane markings
        self.render_lane_markings(road_config, center_x, center_y, window_width, window_height)?;

        Ok(())
    }

    /// Render lane markings (dashed lines)
    fn render_lane_markings(
        &mut self,
        road_config: &RoadConfig,
        center_x: i32,
        center_y: i32,
        window_width: u32,
        window_height: u32,
    ) -> Result<(), String> {
        self.canvas.set_draw_color(Color::RGB(255, 255, 255)); // White for lane markings

        let lane_width = road_config.lane_width as i32;
        let dash_length = road_config.dash_length as i32;
        let dash_gap = dash_length;

        // Horizontal lane markings (for vertical road)
        for lane_offset in [-lane_width, lane_width] {
            let y = center_y + lane_offset;
            let mut x = 0;
            while x < window_width as i32 {
                let dash_rect = Rect::new(x, y - 1, dash_length as u32, 2);
                self.canvas.fill_rect(dash_rect).map_err(|e| e.to_string())?;
                x += dash_length + dash_gap;
            }
        }

        // Vertical lane markings (for horizontal road)
        for lane_offset in [-lane_width, lane_width] {
            let x = center_x + lane_offset;
            let mut y = 0;
            while y < window_height as i32 {
                let dash_rect = Rect::new(x - 1, y, 2, dash_length as u32);
                self.canvas.fill_rect(dash_rect).map_err(|e| e.to_string())?;
                y += dash_length + dash_gap;
            }
        }

        Ok(())
    }

    /// Render a single vehicle
    pub fn render_vehicle(
        &mut self,
        vehicle: &Vehicle,
        vehicle_assets: &VehicleAssets,
        window_width: u32,
        window_height: u32,
    ) -> Result<(), String> {
        if vehicle.texture_index >= vehicle_assets.texture_count() {
            return Ok(()); // Skip if texture index is invalid
        }

        let center_x = window_width as f32 / 2.0;
        let center_y = window_height as f32 / 2.0;

        // Convert world coordinates to screen coordinates
        let screen_x = (center_x + vehicle.position.x) as i32;
        let screen_y = (center_y - vehicle.position.y) as i32; // Flip Y axis

        // Vehicle size (80x60 pixels as per user's manual change)
        let vehicle_width = 80;
        let vehicle_height = 60;

        // Create destination rectangle
        let dst_rect = Rect::new(
            screen_x - vehicle_width / 2,
            screen_y - vehicle_height / 2,
            vehicle_width as u32,
            vehicle_height as u32,
        );

        // Get the texture for this vehicle
        if let Some(texture) = vehicle_assets.car_textures.get(vehicle.texture_index) {
            // Calculate rotation angle in degrees
            let angle_degrees = vehicle.rotation * 180.0 / std::f32::consts::PI;
            
            // Render the rotated texture
            self.canvas
                .copy_ex(
                    texture,
                    None,
                    dst_rect,
                    angle_degrees as f64,
                    None,
                    false,
                    false,
                )
                .map_err(|e| e.to_string())?;
        }

        Ok(())
    }

    /// Render all vehicles
    pub fn render_vehicles(
        &mut self,
        vehicles: &[Vehicle],
        vehicle_assets: &VehicleAssets,
    ) -> Result<(), String> {
        let (window_width, window_height) = self.canvas.output_size().map_err(|e| e.to_string())?;

        for vehicle in vehicles {
            self.render_vehicle(vehicle, vehicle_assets, window_width, window_height)?;
        }

        Ok(())
    }
}
