/// Pure geometry and helper functions.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

/// Return the center line for a lane index (simple horizontal lanes example).
pub fn lane_center(index: usize, lane_width: f32) -> Point {
    Point {
        x: 0.0,
        y: (index as f32 + 0.5) * lane_width,
    }
}

pub fn distance(a: Point, b: Point) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    (dx * dx + dy * dy).sqrt()
}

/// Simple conflict detector: two points conflict if closer than `sep`.
pub fn conflict(a: Point, b: Point, sep: f32) -> bool {
    distance(a, b) < sep
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lane_center_and_distance() {
        let c0 = lane_center(0, 10.0);
        let c1 = lane_center(1, 10.0);
        assert_eq!(c0, Point { x: 0.0, y: 5.0 });
        assert_eq!(c1, Point { x: 0.0, y: 15.0 });
        assert!((distance(c0, c1) - 10.0).abs() < 1e-6);
    }

    #[test]
    fn conflict_detects_close_points() {
        let a = Point { x: 0.0, y: 0.0 };
        let b = Point { x: 1.0, y: 1.0 };
        assert!(conflict(a, b, 2.0));
        assert!(!conflict(a, b, 1.0));
    }
}
