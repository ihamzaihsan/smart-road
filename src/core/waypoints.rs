use crate::components::{Direction, Route};
use crate::resources::{vehicle_radius, INTERSECTION_HALF, LANE_WIDTH, SAFETY_GAP};
use nalgebra::Vector2;
use std::f32::consts::{FRAC_PI_2, PI};

/// One immutable route, sampled into a polyline with scalar distance along it.
#[derive(Debug)]
pub struct Track {
    pub direction: Direction,
    pub route: Route,
    pub points: Vec<Vector2<f32>>,
    pub cumulative: Vec<f32>,
    pub stop_progress: f32,
    pub clear_progress: f32,
    pub length: f32,
    pub reserved_path: Vec<Vector2<f32>>,
}

impl Track {
    pub fn new(direction: Direction, lane: usize) -> Self {
        let route = Route::ALL[lane];
        let lane_center = LANE_WIDTH * (lane as f32 + 0.5);
        let spawn_distance = match direction {
            Direction::North | Direction::South => 380.0,
            Direction::East | Direction::West => 580.0,
        };
        let exit_distance = match (direction, route) {
            (Direction::North | Direction::South, Route::Straight)
            | (Direction::East | Direction::West, Route::Left | Route::Right) => 420.0,
            _ => 620.0,
        };
        let boundary = INTERSECTION_HALF + vehicle_radius() + SAFETY_GAP;
        let mut points = vec![
            Vector2::new(lane_center, -spawn_distance),
            Vector2::new(lane_center, -boundary),
            Vector2::new(lane_center, -INTERSECTION_HALF),
        ];
        match route {
            Route::Straight => points.push(Vector2::new(lane_center, boundary)),
            Route::Left | Route::Right => {
                let (center, radius, start_angle, end_angle) = if route == Route::Left {
                    (
                        Vector2::new(-INTERSECTION_HALF, -INTERSECTION_HALF),
                        INTERSECTION_HALF + lane_center,
                        0.0,
                        FRAC_PI_2,
                    )
                } else {
                    (
                        Vector2::new(INTERSECTION_HALF, -INTERSECTION_HALF),
                        INTERSECTION_HALF - lane_center,
                        PI,
                        FRAC_PI_2,
                    )
                };
                // Short segments make headings change gradually around a turn.
                let segments = (radius * FRAC_PI_2 / 4.0).ceil() as usize;
                for step in 1..=segments {
                    let angle =
                        start_angle + (end_angle - start_angle) * step as f32 / segments as f32;
                    points.push(center + Vector2::new(angle.cos(), angle.sin()) * radius);
                }
                let exit_sign = if route == Route::Left { -1.0 } else { 1.0 };
                points.push(Vector2::new(exit_sign * boundary, points.last().unwrap().y));
            }
        }
        let clear_index = points.len() - 1;
        let last = points[clear_index];
        points.push(match route {
            Route::Straight => Vector2::new(last.x, exit_distance),
            Route::Left => Vector2::new(-exit_distance, last.y),
            Route::Right => Vector2::new(exit_distance, last.y),
        });
        for point in &mut points {
            *point = direction.rotate(*point);
        }
        let mut cumulative = vec![0.0];
        for segment in points.windows(2) {
            cumulative.push(cumulative.last().unwrap() + (segment[1] - segment[0]).norm());
        }
        Self {
            direction,
            route,
            stop_progress: cumulative[1],
            clear_progress: cumulative[clear_index],
            length: *cumulative.last().unwrap(),
            reserved_path: points[1..=clear_index].to_vec(),
            points,
            cumulative,
        }
    }

    pub fn pose(&self, progress: f32) -> (Vector2<f32>, f32) {
        let index = self
            .cumulative
            .partition_point(|&distance| distance <= progress)
            .saturating_sub(1)
            .min(self.points.len() - 2);
        let offset = self.points[index + 1] - self.points[index];
        let length = offset.norm();
        let fraction = ((progress - self.cumulative[index]) / length).clamp(0.0, 1.0);
        (
            self.points[index] + offset * fraction,
            FRAC_PI_2 - offset.y.atan2(offset.x),
        )
    }
}
