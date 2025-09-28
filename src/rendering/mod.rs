use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{Canvas, TextureCreator};
use sdl2::video::{Window, WindowContext};

use crate::components::Vehicle;
use crate::resources::{RoadConfig, VehicleAssets};

pub struct Renderer<'a> {
    pub canvas: Canvas<Window>,
    pub texture_creator: &'a TextureCreator<WindowContext>,
}

impl<'a> Renderer<'a> {
    pub fn new(canvas: Canvas<Window>, texture_creator: &'a TextureCreator<WindowContext>) -> Self {
        Self {
            canvas,
            texture_creator,
        }
    }

    pub fn clear(&mut self) {
        self.canvas.set_draw_color(Color::RGB(50, 50, 50)); // Dark gray background
        self.canvas.clear();
    }

    pub fn present(&mut self) {
        self.canvas.present();
    }

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
        self.canvas
            .fill_rect(road_rect)
            .map_err(|e| e.to_string())?;

        // Draw vertical road
        let road_rect = Rect::new(
            center_x - (road_width / 2.0) as i32,
            0,
            road_width as u32,
            window_height,
        );
        self.canvas
            .fill_rect(road_rect)
            .map_err(|e| e.to_string())?;

        // Draw lane markings
        self.render_lane_markings(road_config, center_x, center_y, window_width, window_height)?;

        // Draw lane separation lines at intersection boundaries
        self.render_lane_separation_lines(road_config, center_x, center_y)?;

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

        // Calculate intersection bounds (no dashes in center)
        let intersection_half = road_width / 2;
        
        // Calculate separation line position for north direction lanes (which approach from the south)
        let separation_line_buffer = dash_length + dash_gap; // Buffer zone around separation lines
        let south_separation_y = center_y - intersection_half; // North-bound lanes approach this line

        // Calculate all lane positions from left edge
        let left_edge = -road_width / 2;
        let mut lane_separators: Vec<i32> = Vec::new();

        // Add separators between individual lanes (but not between opposing directions)
        // Left group separators (lanes 0|1 and 1|2)
        for j in 1..lanes_per_direction {
            lane_separators.push(left_edge + (j as i32) * lane_width);
        }

        // Right group separators (lanes 3|4 and 4|5)
        for j in (lanes_per_direction + 1)..total_lanes {
            lane_separators.push(left_edge + (j as i32) * lane_width);
        }

        // Draw white dashed lines between individual lanes
        self.canvas.set_draw_color(Color::RGB(255, 255, 255)); // White for lane markings

        for &separator_offset in &lane_separators {
            // Horizontal dashes (for vertical road lanes)
            let y = center_y + separator_offset;
            let mut x = 0;
            while x < window_width as i32 {
                // Skip dashes in intersection area
                if (x - center_x).abs() > intersection_half {
                    let dash_rect = Rect::new(x, y - 1, dash_length as u32, 2);
                    self.canvas
                        .fill_rect(dash_rect)
                        .map_err(|e| e.to_string())?;
                }
                x += dash_length + dash_gap;
            }

            // Vertical dashes (for horizontal road lanes)
            let x = center_x + separator_offset;
            let mut y = 0;
            while y < window_height as i32 {
                // Skip dashes in intersection area
                if (y - center_y).abs() > intersection_half {
                    // Skip dashes near south separation line (where north-bound lanes approach)
                    let near_south_line = (y - south_separation_y).abs() < separation_line_buffer;
                    
                    if !near_south_line {
                        let dash_rect = Rect::new(x - 1, y, 2, dash_length as u32);
                        self.canvas
                            .fill_rect(dash_rect)
                            .map_err(|e| e.to_string())?;
                    }
                }
                y += dash_length + dash_gap;
            }
        }

        // Draw white solid median separators between opposing directions (skip intersection)
        self.canvas.set_draw_color(Color::RGB(255, 255, 255)); // White for medians
        let median_width = 6;

        // Horizontal median separator (separates up/down traffic) - skip intersection
        // Left side of intersection
        let left_median_rect = Rect::new(
            0,
            center_y - median_width / 2,
            (center_x - intersection_half) as u32,
            median_width as u32,
        );
        self.canvas
            .fill_rect(left_median_rect)
            .map_err(|e| e.to_string())?;

        // Right side of intersection
        let right_median_rect = Rect::new(
            center_x + intersection_half,
            center_y - median_width / 2,
            (center_x - intersection_half) as u32,
            median_width as u32,
        );
        self.canvas
            .fill_rect(right_median_rect)
            .map_err(|e| e.to_string())?;

        // Vertical median separator (separates left/right traffic) - skip intersection
        // Top side of intersection
        let top_median_rect = Rect::new(
            center_x - median_width / 2,
            0,
            median_width as u32,
            (center_y - intersection_half) as u32,
        );
        self.canvas
            .fill_rect(top_median_rect)
            .map_err(|e| e.to_string())?;

        // Bottom side of intersection
        let bottom_median_rect = Rect::new(
            center_x - median_width / 2,
            center_y + intersection_half,
            median_width as u32,
            (center_y - intersection_half) as u32,
        );
        self.canvas
            .fill_rect(bottom_median_rect)
            .map_err(|e| e.to_string())?;

        Ok(())
    }

    fn render_lane_separation_lines(
        &mut self,
        road_config: &RoadConfig,
        center_x: i32,
        center_y: i32,
    ) -> Result<(), String> {
        let lane_width = road_config.lane_width as i32;
        let lanes_per_direction = road_config.num_lanes;
        let intersection_half = (lane_width * lanes_per_direction as i32 * 2) / 2;

        // Line thickness
        let line_thickness = 3;

        self.canvas.set_draw_color(Color::RGB(255, 255, 255));

        // East direction spawning lanes (coming from left, going right)
        // These are the bottom 3 lanes in the vertical road
        let east_line_x = center_x - intersection_half;
        let east_line_start_y = center_y; // Start at center (after median)
        let east_line_end_y = center_y + (lanes_per_direction as i32 * lane_width);
        
        for y in east_line_start_y..=east_line_end_y {
            let line_rect = Rect::new(east_line_x - line_thickness / 2, y, line_thickness as u32, 1);
            self.canvas.fill_rect(line_rect).map_err(|e| e.to_string())?;
        }

        // West direction spawning lanes (coming from right, going left)
        // These are the top 3 lanes in the vertical road
        let west_line_x = center_x + intersection_half;
        let west_line_start_y = center_y - (lanes_per_direction as i32 * lane_width);
        let west_line_end_y = center_y;
        
        for y in west_line_start_y..=west_line_end_y {
            let line_rect = Rect::new(west_line_x - line_thickness / 2, y, line_thickness as u32, 1);
            self.canvas.fill_rect(line_rect).map_err(|e| e.to_string())?;
        }

        // North direction spawning lanes (coming from bottom, going up)
        // These are the right 3 lanes in the horizontal road
        let north_line_y = center_y + intersection_half;
        let north_line_start_x = center_x; // Start at center (after median)
        let north_line_end_x = center_x + (lanes_per_direction as i32 * lane_width);
        
        for x in north_line_start_x..=north_line_end_x {
            let line_rect = Rect::new(x, north_line_y - line_thickness / 2, 1, line_thickness as u32);
            self.canvas.fill_rect(line_rect).map_err(|e| e.to_string())?;
        }

        // South direction spawning lanes (coming from top, going down)
        // These are the left 3 lanes in the horizontal road
        let south_line_y = center_y - intersection_half;
        let south_line_start_x = center_x - (lanes_per_direction as i32 * lane_width);
        let south_line_end_x = center_x;
        
        for x in south_line_start_x..=south_line_end_x {
            let line_rect = Rect::new(x, south_line_y - line_thickness / 2, 1, line_thickness as u32);
            self.canvas.fill_rect(line_rect).map_err(|e| e.to_string())?;
        }

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

    /// Draw small red squares at each checkpoint for debugging visibility.
    pub fn render_checkpoints(
        &mut self,
        checkpoints: &[nalgebra::Vector2<f32>],
    ) -> Result<(), String> {
        // Use the same mapping as render_vehicle (center of screen = world 0,0; Y flipped)
        let (window_width, window_height) = self.canvas.output_size().map_err(|e| e.to_string())?;
        let center_x = window_width as i32 / 2;
        let center_y = window_height as i32 / 2;
        let size: i32 = 6; // red square size

        self.canvas.set_draw_color(Color::RGB(220, 30, 30));

        for p in checkpoints {
            let sx = center_x + p.x.round() as i32;
            let sy = center_y - p.y.round() as i32;
            let rect = Rect::new(sx - size / 2, sy - size / 2, size as u32, size as u32);
            self.canvas.fill_rect(rect)?;
        }

        Ok(())
    }
}
