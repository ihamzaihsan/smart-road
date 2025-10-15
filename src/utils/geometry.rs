#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

impl Point {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn distance_to(&self, other: Point) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}
pub fn lane_center(index: usize, lane_width: f32) -> Point {
    Point {
        x: 0.0,
        y: (index as f32 + 0.5) * lane_width,
    }
}

pub fn distance(a: Point, b: Point) -> f32 {
    a.distance_to(b)
}

pub fn conflict(a: Point, b: Point, separation: f32) -> bool {
    distance(a, b) < separation
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

    #[test]
    fn point_distance() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(3.0, 4.0);
        assert!((p1.distance_to(p2) - 5.0).abs() < 1e-6);
    }
}
