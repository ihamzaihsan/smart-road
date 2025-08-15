//! Library crate root for smart-road.
//!
//! Re-exports modules and plugins so the binary can stay small.

pub mod prelude;
pub mod components;
pub mod resources;
pub mod systems;
pub mod plugins;
pub mod ui;
pub mod util;

pub use plugins::*;
