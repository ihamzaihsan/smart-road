use nalgebra::Vector2;

/// Represents a vehicle entity in the simulation.
#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub position: Vector2<f32>,
    pub speed: f32,
    pub state: VehicleState,
    pub route: Route,
    pub direction: Direction,
    pub target_speed: f32,
    pub rotation: f32,  // Rotation angle in radians
    pub texture_index: usize,  // Index into the car textures array
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    Straight,
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleState {
    Waiting,
    Moving,
    Stopped,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    North,  // Moving north (up)
    South,  // Moving south (down)
    East,   // Moving east (right)
    West,   // Moving west (left)
}

impl Direction {
    /// Get the movement vector for this direction
    pub fn movement_vector(&self) -> Vector2<f32> {
        match self {
            Direction::North => Vector2::new(0.0, 1.0),
            Direction::South => Vector2::new(0.0, -1.0),
            Direction::East => Vector2::new(1.0, 0.0),
            Direction::West => Vector2::new(-1.0, 0.0),
        }
    }

    /// Get the rotation angle in radians for this direction
    pub fn rotation_angle(&self) -> f32 {
        match self {
            Direction::North => 0.0,
            Direction::South => std::f32::consts::PI,
            Direction::East => -std::f32::consts::PI / 2.0,
            Direction::West => std::f32::consts::PI / 2.0,
        }
    }
}
