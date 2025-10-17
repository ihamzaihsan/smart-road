use nalgebra::Vector2;

fn cross(a: Vector2<f32>, b: Vector2<f32>) -> f32 {
    a.x * b.y - a.y * b.x
}

fn point_segment_distance(point: Vector2<f32>, a: Vector2<f32>, b: Vector2<f32>) -> f32 {
    let segment = b - a;
    let length_squared = segment.norm_squared();
    if length_squared < f32::EPSILON {
        return (point - a).norm();
    }
    let fraction = ((point - a).dot(&segment) / length_squared).clamp(0.0, 1.0);
    (point - (a + segment * fraction)).norm()
}

/// Minimum distance between closed segments, including crossings and zero lengths.
pub fn segment_distance(a: Vector2<f32>, b: Vector2<f32>, c: Vector2<f32>, d: Vector2<f32>) -> f32 {
    let denominator = cross(b - a, d - c);
    if denominator.abs() > f32::EPSILON {
        let t = cross(c - a, d - c) / denominator;
        let u = cross(c - a, b - a) / denominator;
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            return 0.0;
        }
    }
    point_segment_distance(a, c, d)
        .min(point_segment_distance(b, c, d))
        .min(point_segment_distance(c, a, b))
        .min(point_segment_distance(d, a, b))
}
