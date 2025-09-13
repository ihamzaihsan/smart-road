#[cfg(test)]
mod geometry_tests {
    use crate::utils::geometry::{Point, distance, lane_center};

    #[test]
    fn basic_geom_works() {
        let a = Point { x: 0.0, y: 0.0 };
        let b = Point { x: 3.0, y: 4.0 };
        assert_eq!(distance(a, b), 5.0);
        let lc = lane_center(2, 10.0);
        assert_eq!(lc.y, 25.0);
    }
}