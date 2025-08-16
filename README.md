# smart_road (Rust)

This repository is the Smart Road intersection simulator. This crate contains the simulation engine, UI, and assets manager.

Layout created for Rust:

- Cargo.toml - Rust manifest
- src/main.rs - binary entry point
- src/lib.rs - library API and module exports
- src/engine/ - simulation engine code
- src/ui/ - rendering and input
- src/assets/ - asset manager and loaders
- src/types.rs - shared types (Vehicle, Route, Stats)
- tests/ - integration tests
- scripts/ - helper run scripts

How to build and run (Windows PowerShell):

```powershell
cd <project-root>
cargo build --release
cargo run --bin visual_sim
```

## Automatic Traffic Simulation

The smart-road simulator now runs **completely automatically** without requiring user input:

- **Automatic Vehicle Spawning**: Vehicles spawn automatically every 1-3 seconds from random directions
- **Random Direction Selection**: Each spawn randomly selects from North, South, East, or West directions
- **Random Lane Selection**: Vehicles spawn in random lanes within their chosen direction
- **Random Car Sprites**: Each vehicle gets a randomly selected car sprite from the assets directory
- **Continuous Operation**: The simulation runs continuously, creating realistic traffic flow

## Features Implemented

✅ **Road Intersection Rendering**: Procedural cross-intersection with lane markings
✅ **Automatic Vehicle Spawning**: Timer-based automatic spawning system (no user input required)
✅ **Random Direction Selection**: Vehicles spawn from randomly selected directions (North/South/East/West)
✅ **Configurable Spawn Timing**: Automatic spawning every 1-3 seconds with random intervals
✅ **Car Sprite Rendering**: Uses actual car images from assets/ directory (Car.png, Audi.png, Mini_van.png, taxi.png, Police.png)
✅ **Random Lane Selection**: Vehicles spawn in random lanes within their direction
✅ **Vehicle Rotation**: Car sprites are properly rotated based on movement direction
✅ **Vehicle Movement**: Vehicles move in their assigned direction with realistic physics
✅ **Collision Prevention**: Prevents vehicles from spawning on top of each other
✅ **Spawn Cooldown**: 500ms cooldown between spawns in the same direction
✅ **Vehicle Cleanup**: Vehicles are automatically removed when they leave the simulation area

## Next steps:
- Implement intersection control algorithm (scheduler)
- Add vehicle route following (left/right turns)
- Implement collision detection between moving vehicles
- Add statistics tracking and HUD display
- Add manual control toggle (to switch between automatic and manual spawning)
