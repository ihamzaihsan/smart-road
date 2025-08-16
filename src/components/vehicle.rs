use bevy::prelude::*;

/// Represents a vehicle entity in the simulation.
#[derive(Component, Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub speed: f32,
    pub state: VehicleState,
    pub route: Route,
    pub direction: Direction,
    pub target_speed: f32,
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
    pub fn movement_vector(&self) -> Vec2 {
        match self {
            Direction::North => Vec2::new(0.0, 1.0),
            Direction::South => Vec2::new(0.0, -1.0),
            Direction::East => Vec2::new(1.0, 0.0),
            Direction::West => Vec2::new(-1.0, 0.0),
        }
    }
}
