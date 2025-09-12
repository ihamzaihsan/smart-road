use sdl2::render::{Canvas, TextureCreator};
use sdl2::video::{Window, WindowContext};
use sdl2::rect::Rect;
use sdl2::pixels::Color;
use crate::components::Vehicle;
use crate::resources::{RoadConfig, VehicleAssets};
pub struct Renderer<'a> {
    pub canvas: Canvas<Window>,
    pub texture_creator: &'a TextureCreator<WindowContext>,
}
impl<'a> Renderer<'a> {
    pub fn new(canvas: Canvas<Window>, texture_creator: &'a TextureCreator<WindowContext>) -> Self {
        Self { canvas, texture_creator }
    }
    pub fn clear(&mut self) {
        self.canvas.set_draw_color(Color::RGB(50, 50, 50));
        self.canvas.clear();
    }
    pub fn present(&mut self) {
        self.canvas.present();
    }
    pub fn render_roads(&mut self, road_config: &RoadConfig) -> Result<(), String> {
        let (window_width, window_height) = self.canvas.output_size().map_err(|e| e.to_string())?;
        let center_x = window_width as i32 / 2;
        let center_y = window_height as i32 / 2;
        let road_width = road_config.lane_width * road_config.num_lanes as f32 * 2.0;
        self.canvas.set_draw_color(Color::RGB(80, 80, 80));
        let road_rect = Rect::new(
            0,
            center_y - (road_width / 2.0) as i32,
            window_width,
            road_width as u32,
        );
        self.canvas.fill_rect(road_rect).map_err(|e| e.to_string())?;
        let road_rect = Rect::new(
            center_x - (road_width / 2.0) as i32,
            0,
            road_width as u32,
            window_height,
        );
        self.canvas.fill_rect(road_rect).map_err(|e| e.to_string())?;
        self.render_lane_markings(road_config, center_x, center_y, window_width, window_height)?;
        Ok(())
    }
    fn render_lane_markings(
        &mut self,
        road_config: &RoadConfig,
        center_x: i32,
        center_y: i32,
        window_width: u32,
        window_height: u32,
    ) -> Result<(), String> {
        let lane_width = road_config.lane_width as i32;
        let lanes_per_direction = road_config.num_lanes;
        let total_lanes = lanes_per_direction * 2;
        let road_width = lane_width * total_lanes as i32;
        let dash_length = road_config.dash_length as i32;
        let dash_gap = road_config.dash_gap as i32;
        let intersection_half = road_width / 2;
        let left_edge = -road_width / 2;
        let mut lane_separators: Vec<i32> = Vec::new();
        for j in 1..lanes_per_direction {
            lane_separators.push(left_edge + (j as i32) * lane_width);
        }
        for j in (lanes_per_direction + 1)..total_lanes {
            lane_separators.push(left_edge + (j as i32) * lane_width);
        }
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        for &separator_offset in &lane_separators {
            let y = center_y + separator_offset;
            let mut x = 0;
            while x < window_width as i32 {
                if (x - center_x).abs() > intersection_half {
                    let dash_rect = Rect::new(x, y - 1, dash_length as u32, 2);
                    self.canvas.fill_rect(dash_rect).map_err(|e| e.to_string())?;
                }
                x += dash_length + dash_gap;
            }
            let x = center_x + separator_offset;
            let mut y = 0;
            while y < window_height as i32 {
                if (y - center_y).abs() > intersection_half {
                    let dash_rect = Rect::new(x - 1, y, 2, dash_length as u32);
                    self.canvas.fill_rect(dash_rect).map_err(|e| e.to_string())?;
                }
                y += dash_length + dash_gap;
            }
        }
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        let median_width = 6;
        let left_median_rect = Rect::new(
            0,
            center_y - median_width / 2,
            (center_x - intersection_half) as u32,
            median_width as u32,
        );
        self.canvas.fill_rect(left_median_rect).map_err(|e| e.to_string())?;
        let right_median_rect = Rect::new(
            center_x + intersection_half,
            center_y - median_width / 2,
            (center_x - intersection_half) as u32,
            median_width as u32,
        );
        self.canvas.fill_rect(right_median_rect).map_err(|e| e.to_string())?;
        let top_median_rect = Rect::new(
            center_x - median_width / 2,
            0,
            median_width as u32,
            (center_y - intersection_half) as u32,
        );
        self.canvas.fill_rect(top_median_rect).map_err(|e| e.to_string())?;
        let bottom_median_rect = Rect::new(
            center_x - median_width / 2,
            center_y + intersection_half,
            median_width as u32,
            (center_y - intersection_half) as u32,
        );
        self.canvas.fill_rect(bottom_median_rect).map_err(|e| e.to_string())?;
        Ok(())
    }
    pub fn render_vehicle(
        &mut self,
        vehicle: &Vehicle,
        vehicle_assets: &VehicleAssets,
        window_width: u32,
        window_height: u32,
    ) -> Result<(), String> {
        if vehicle.texture_index >= vehicle_assets.texture_count() {
            return Ok(());
        }
        let center_x = window_width as f32 / 2.0;
        let center_y = window_height as f32 / 2.0;
        let screen_x = (center_x + vehicle.position.x) as i32;
        let screen_y = (center_y - vehicle.position.y) as i32;
        let vehicle_width = 70;
        let vehicle_height = 50;
        let dst_rect = Rect::new(
            screen_x - vehicle_width / 2,
            screen_y - vehicle_height / 2,
            vehicle_width as u32,
            vehicle_height as u32,
        );
        if let Some(texture) = vehicle_assets.car_textures.get(vehicle.texture_index) {
            let angle_degrees = vehicle.rotation * 180.0 / std::f32::consts::PI;
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
    pub fn render_checkpoints(&mut self, checkpoints: &[nalgebra::Vector2<f32>]) -> Result<(), String> {
        let (window_width, window_height) = self.canvas.output_size().map_err(|e| e.to_string())?;
        let center_x = window_width as i32 / 2;
        let center_y = window_height as i32 / 2;
        let size: i32 = 6;
        self.canvas.set_draw_color(Color::RGB(220, 30, 30));
        for p in checkpoints {
            let sx = center_x + p.x.round() as i32;
            let sy = center_y - p.y.round() as i32;
            let rect = Rect::new(sx - size/2, sy - size/2, size as u32, size as u32);
            self.canvas.fill_rect(rect)?;
        }
        Ok(())
    }
}
