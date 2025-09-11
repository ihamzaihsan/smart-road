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
impl From<(f32, f32)> for Point {
    fn from(item: (f32, f32)) -> Self {
        Point { x: item.0, y: item.1 }
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
pub fn angle_between_points(from: Point, to: Point) -> f32 {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    dy.atan2(dx)
}
pub fn rotate_point(point: Point, angle: f32) -> Point {
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    Point {
        x: point.x * cos_a - point.y * sin_a,
        y: point.x * sin_a + point.y * cos_a,
    }
}
pub fn line_intersection(p1: Point, p2: Point, p3: Point, p4: Point) -> Option<Point> {
    let den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if den == 0.0 {
        return None; // Lines are parallel
    }
    let t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / den;
    let u = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) / den;

    if t > 0.0 && t < 1.0 && u > 0.0 && u < 1.0 {
        Some(Point {
            x: p1.x + t * (p2.x - p1.x),
            y: p1.y + t * (p2.y - p1.y),
        })
    } else {
        None
    }
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
    #[test]
    fn line_intersection_test() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(2.0, 2.0);
        let p3 = Point::new(0.0, 2.0);
        let p4 = Point::new(2.0, 0.0);
        let intersection = line_intersection(p1, p2, p3, p4);
        assert!(intersection.is_some());
        let ip = intersection.unwrap();
        assert!((ip.x - 1.0).abs() < 1e-6);
        assert!((ip.y - 1.0).abs() < 1e-6);
    }
}
