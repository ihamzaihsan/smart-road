use nalgebra::Vector2;
#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub position: Vector2<f32>,
    pub speed: f32,
    pub state: VehicleState,
    pub route: Route,
    pub direction: Direction,
    pub target_speed: f32,
    pub rotation: f32,
    pub texture_index: usize,
    pub path: Vec<Vector2<f32>>,
    pub path_index: usize,
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
}
impl Vehicle {
    pub fn get_current_waypoint(&self) -> Option<Vector2<f32>> {
        if self.path_index < self.path.len() {
            Some(self.path[self.path_index])
        } else {
            None
        }
    }
    pub fn advance_waypoint(&mut self) {
        if self.path_index < self.path.len() {
            self.path_index += 1;
        }
    }
    pub fn has_reached_destination(&self) -> bool {
        self.path_index >= self.path.len()
    }
    pub fn set_path(&mut self, path: Vec<Vector2<f32>>) {
        self.path = path;
        self.path_index = 0;
    }
}
