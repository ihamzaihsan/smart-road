mod font;

use crate::components::{Direction, Route, VehicleState};
use crate::core::Simulation;
use crate::resources::{
    assets::VehicleAssets, INTERSECTION_HALF, LANE_WIDTH, VEHICLE_LENGTH, VEHICLE_WIDTH,
};
use nalgebra::Vector2;
use sdl2::{
    pixels::Color,
    rect::{Point, Rect},
    render::{Canvas, TextureCreator},
    video::{Window, WindowContext},
};

const TEXT: Color = Color::RGB(222, 232, 235);
const MUTED: Color = Color::RGB(133, 159, 164);
const ACCENT: Color = Color::RGB(73, 192, 178);

pub struct Renderer<'a> {
    canvas: Canvas<Window>,
    assets: VehicleAssets<'a>,
}

impl<'a> Renderer<'a> {
    pub fn new(
        canvas: Canvas<Window>,
        creator: &'a TextureCreator<WindowContext>,
    ) -> Result<Self, String> {
        Ok(Self {
            canvas,
            assets: VehicleAssets::new(creator)?,
        })
    }

    pub fn window(&self) -> &Window {
        self.canvas.window()
    }

    fn rectangle(&mut self, rect: Rect, color: Color) -> Result<(), String> {
        self.canvas.set_draw_color(color);
        self.canvas.fill_rect(rect)
    }

    fn text(&mut self, text: &str, x: i32, y: i32, scale: u32, color: Color) -> Result<(), String> {
        font::draw(&mut self.canvas, text, x, y, scale, color)
    }

    fn world(point: Vector2<f32>) -> Point {
        Point::new(
            (600.0 + point.x).round() as i32,
            (400.0 - point.y).round() as i32,
        )
    }

    fn roads(&mut self) -> Result<(), String> {
        let half = INTERSECTION_HALF as i32;
        let width = (half * 2) as u32;
        self.rectangle(Rect::new(444, 0, width + 12, 800), Color::RGB(47, 65, 66))?;
        self.rectangle(Rect::new(0, 244, 1200, width + 12), Color::RGB(47, 65, 66))?;
        self.rectangle(Rect::new(450, 0, width, 800), Color::RGB(42, 51, 61))?;
        self.rectangle(Rect::new(0, 250, 1200, width), Color::RGB(42, 51, 61))?;
        let lane = LANE_WIDTH as i32;
        for separator in [-2 * lane, -lane, lane, 2 * lane] {
            for x in (0..1200).step_by(32) {
                if !(430..=750).contains(&x) {
                    self.rectangle(Rect::new(x, 400 + separator, 18, 2), MUTED)?;
                }
            }
            for y in (0..800).step_by(32) {
                if !(230..=550).contains(&y) {
                    self.rectangle(Rect::new(600 + separator, y, 2, 18), MUTED)?;
                }
            }
        }
        let median = Color::RGB(219, 183, 98);
        for rect in [
            Rect::new(0, 398, 450, 4),
            Rect::new(750, 398, 450, 4),
            Rect::new(598, 0, 4, 250),
            Rect::new(598, 550, 4, 250),
        ] {
            self.rectangle(rect, median)?;
        }
        // Stop markings cover only incoming lanes.
        for rect in [
            Rect::new(600, 549, 150, 3),
            Rect::new(450, 248, 150, 3),
            Rect::new(448, 400, 3, 150),
            Rect::new(749, 250, 3, 150),
        ] {
            self.rectangle(rect, TEXT)?;
        }
        self.canvas.set_draw_color(MUTED);
        for direction in Direction::ALL {
            for (lane, route) in Route::ALL.into_iter().enumerate() {
                let x = (lane as f32 + 0.5) * LANE_WIDTH;
                let base = Vector2::new(x, -230.0);
                let top = base + Vector2::new(0.0, 20.0);
                let arrow = match route {
                    Route::Straight => top,
                    Route::Left => top + Vector2::new(-12.0, 0.0),
                    Route::Right => top + Vector2::new(12.0, 0.0),
                };
                self.canvas.draw_line(
                    Self::world(direction.rotate(base)),
                    Self::world(direction.rotate(top)),
                )?;
                self.canvas.draw_line(
                    Self::world(direction.rotate(top)),
                    Self::world(direction.rotate(arrow)),
                )?;
                let tip_a = if route == Route::Straight {
                    Vector2::new(-5.0, -5.0)
                } else if route == Route::Left {
                    Vector2::new(5.0, 5.0)
                } else {
                    Vector2::new(-5.0, 5.0)
                };
                let tip_b = if route != Route::Right {
                    Vector2::new(5.0, -5.0)
                } else {
                    Vector2::new(-5.0, -5.0)
                };
                for tip in [tip_a, tip_b] {
                    self.canvas.draw_line(
                        Self::world(direction.rotate(arrow)),
                        Self::world(direction.rotate(arrow + tip)),
                    )?;
                }
            }
        }
        Ok(())
    }

    fn panels(&mut self, simulation: &Simulation) -> Result<(), String> {
        for rect in [
            Rect::new(24, 24, 394, 194),
            Rect::new(782, 24, 394, 194),
            Rect::new(24, 582, 394, 194),
            Rect::new(782, 582, 394, 194),
        ] {
            self.rectangle(rect, Color::RGB(23, 37, 43))?;
        }
        self.text("SMARTROAD", 42, 42, 4, TEXT)?;
        self.text("TRAFFIC INTERSECTION SIMULATION", 44, 84, 2, MUTED)?;
        self.text("NO LIGHTS. COORDINATED CROSSINGS.", 44, 116, 1, MUTED)?;
        let mode = if simulation.automatic {
            "AUTOMATIC TRAFFIC"
        } else {
            "MANUAL TRAFFIC"
        };
        self.text(mode, 44, 146, 2, ACCENT)?;
        self.text(
            &format!("SIMULATION TIME: {:.1} S", simulation.time),
            44,
            180,
            1,
            TEXT,
        )?;

        let summary = simulation.summary();
        self.text("LIVE METRICS", 802, 42, 2, ACCENT)?;
        for (index, line) in [
            format!("COMPLETED: {}", summary.total_vehicles_passed),
            format!(
                "ACTIVE: {}   WAITING: {}",
                summary.active_vehicles,
                simulation
                    .vehicles
                    .iter()
                    .filter(|v| v.state == VehicleState::Waiting)
                    .count()
            ),
            format!("THROUGHPUT: {:.1}/MIN", summary.throughput_per_minute),
            format!("CLOSE CALLS: {}", summary.close_calls),
            format!("COLLISIONS: {}", summary.collisions),
        ]
        .into_iter()
        .enumerate()
        {
            self.text(&line, 802, 78 + index as i32 * 25, 2, TEXT)?;
        }

        self.text("CONTROLS", 44, 600, 2, ACCENT)?;
        for (index, line) in [
            "ARROW KEYS   SPAWN FROM EACH APPROACH",
            "R            TOGGLE AUTOMATIC TRAFFIC",
            "ESC          END AND VIEW STATISTICS",
            "LANES        LEFT / STRAIGHT / RIGHT",
        ]
        .into_iter()
        .enumerate()
        {
            self.text(line, 44, 636 + index as i32 * 25, 1, TEXT)?;
        }
        self.text("RESERVATION CONTROLLER", 802, 600, 2, ACCENT)?;
        self.text(
            &format!(
                "ACTIVE RESERVATIONS: {}",
                simulation.intersection.active_count()
            ),
            802,
            638,
            1,
            TEXT,
        )?;
        self.text("COMPATIBLE ROUTES CROSS TOGETHER", 802, 663, 1, MUTED)?;
        self.text("CONFLICTING ROUTES WAIT THEIR TURN", 802, 688, 1, MUTED)?;
        self.text("5 VEHICLE STYLES / VARIABLE BRAKING", 802, 713, 1, MUTED)?;
        self.text("120 HZ PHYSICS / 60 FPS DISPLAY", 802, 746, 1, MUTED)?;
        Ok(())
    }

    pub fn render(&mut self, simulation: &Simulation) -> Result<(), String> {
        self.canvas.set_draw_color(Color::RGB(28, 48, 47));
        self.canvas.clear();
        self.roads()?;
        for vehicle in &simulation.vehicles {
            let center = Self::world(vehicle.position);
            let rect = Rect::new(
                center.x - VEHICLE_WIDTH as i32 / 2,
                center.y - VEHICLE_LENGTH as i32 / 2,
                VEHICLE_WIDTH as u32,
                VEHICLE_LENGTH as u32,
            );
            self.canvas.copy_ex(
                &self.assets.textures[vehicle.style],
                None,
                rect,
                vehicle.rotation.to_degrees() as f64,
                None,
                false,
                false,
            )?;
        }
        self.panels(simulation)?;
        self.canvas.present();
        Ok(())
    }
}
