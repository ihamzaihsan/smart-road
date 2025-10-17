//! Deterministic traffic simulation with an optional SDL2 presentation layer.
pub mod components;
pub mod core;
#[cfg(feature = "graphics")]
pub mod rendering;
pub mod resources;

pub const PROJECT_NAME: &str = "SmartRoad: Traffic Intersection Simulation";
