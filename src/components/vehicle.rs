use bevy::prelude::*;

/// Represents a vehicle entity in the simulation.
#[derive(Component, Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub speed: f32,
    pub state: VehicleState,
    pub route: Route,
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
