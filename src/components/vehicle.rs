use nalgebra::Vector2;

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub position: Vector2<f32>,
    pub width: f32,
    pub length: f32,
    pub speed: f32,
    pub state: VehicleState,
    pub route: Route,
    pub lane_index: usize,
    pub direction: Direction,
    pub target_speed: f32,
    pub rotation: f32,
    pub texture_index: usize,
    pub path: Vec<Vector2<f32>>,
    pub path_index: usize,
    pub queue_target: Option<Vector2<f32>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
    Yielding, // Added for conflict resolution
    AlwaysMoving, // For right-turn vehicles that don't create intersection conflicts
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    North,
    South,
    East,
    West,
}

impl Direction {
    pub fn movement_vector(&self) -> Vector2<f32> {
        match self {
            Direction::North => Vector2::new(0.0, 1.0),
            Direction::South => Vector2::new(0.0, -1.0),
            Direction::East => Vector2::new(1.0, 0.0),
            Direction::West => Vector2::new(-1.0, 0.0),
        }
    }

    pub fn rotation_angle(&self) -> f32 {
        match self {
            Direction::North => 0.0,
            Direction::South => std::f32::consts::PI,
            Direction::East => std::f32::consts::PI / 2.0,
            Direction::West => -std::f32::consts::PI / 2.0,
        }
    }

    pub fn turn_left(&self) -> Self {
        match self {
            Direction::North => Direction::West,
            Direction::South => Direction::East,
            Direction::East => Direction::North,
            Direction::West => Direction::South,
        }
    }

    pub fn turn_right(&self) -> Self {
        match self {
            Direction::North => Direction::East,
            Direction::South => Direction::West,
            Direction::East => Direction::South,
            Direction::West => Direction::North,
        }
    }

    pub fn from_vector(vector: Vector2<f32>) -> Option<Self> {
        let mag_sq = vector.x * vector.x + vector.y * vector.y;
        if mag_sq < 1e-4 {
            return None;
        }
        if vector.x.abs() > vector.y.abs() {
            if vector.x > 0.0 {
                Some(Direction::East)
            } else {
                Some(Direction::West)
            }
        } else if vector.y.abs() > vector.x.abs() {
            if vector.y > 0.0 {
                Some(Direction::North)
            } else {
                Some(Direction::South)
            }
        } else {
            None
        }
    }
}

impl Vehicle {
    /// Gets the current waypoint the vehicle is heading towards
    pub fn get_current_waypoint(&self) -> Option<Vector2<f32>> {
        if self.path_index < self.path.len() {
            Some(self.path[self.path_index])
        } else {
            None
        }
    }

    /// Advances to the next waypoint
    pub fn advance_waypoint(&mut self) {
        if self.path_index < self.path.len() {
            self.path_index += 1;
        }
    }

    /// Checks if the vehicle has reached its destination
    pub fn has_reached_destination(&self) -> bool {
        self.path_index >= self.path.len()
    }

    /// Sets a new path for the vehicle
    pub fn set_path(&mut self, path: Vec<Vector2<f32>>) {
        self.path = path;
        self.path_index = 0;
    }
}
