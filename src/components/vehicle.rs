use nalgebra::Vector2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    North,
    South,
    East,
    West,
}

impl Direction {
    pub const ALL: [Self; 4] = [Self::North, Self::South, Self::East, Self::West];

    pub fn index(self) -> usize {
        match self {
            Self::North => 0,
            Self::South => 1,
            Self::East => 2,
            Self::West => 3,
        }
    }

    /// Rotate a northbound route into this approach. World Y points north.
    pub fn rotate(self, point: Vector2<f32>) -> Vector2<f32> {
        match self {
            Self::North => point,
            Self::South => -point,
            Self::East => Vector2::new(point.y, -point.x),
            Self::West => Vector2::new(-point.y, point.x),
        }
    }
}

/// Lanes are ordered from the median outwards; routes never change after spawning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    Left,
    Straight,
    Right,
}

impl Route {
    pub const ALL: [Self; 3] = [Self::Left, Self::Straight, Self::Right];
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleState {
    Approaching,
    Waiting,
    Crossing,
    Exiting,
}

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub track_index: usize,
    pub position: Vector2<f32>,
    pub rotation: f32,
    pub speed: f32,
    pub progress: f32,
    pub state: VehicleState,
    pub style: usize,
    pub acceleration: f32,
    pub deceleration: f32,
    pub detected_at: f64,
    pub waiting_time: f64,
    pub admitted: bool,
}
