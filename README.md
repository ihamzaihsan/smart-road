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

## Hybrid Vehicle Spawning System

The smart-road simulator supports both **manual and automatic vehicle spawning**:

### Manual Controls (Always Active)
- **Arrow Up**: Spawn vehicle from south moving north (random car sprite, fixed lane)
- **Arrow Down**: Spawn vehicle from north moving south (random car sprite, fixed lane)
- **Arrow Right**: Spawn vehicle from west moving east (random car sprite, fixed lane)
- **Arrow Left**: Spawn vehicle from east moving west (random car sprite, fixed lane)

### Automatic Spawning Toggle
- **R Key**: Toggle automatic spawning on/off
  - **Default**: Automatic spawning is **disabled** (manual-only mode)
  - **Press R**: Enable automatic spawning (vehicles spawn every 1 second from random directions)
  - **Press R again**: Disable automatic spawning (return to manual-only mode)
  - **Console feedback**: Shows current automatic spawning status

### Hybrid Mode
When automatic spawning is enabled, both systems work simultaneously:
- Vehicles spawn automatically every 1 second from random directions with random lanes
- Arrow keys continue to work for immediate manual spawning
- Both systems use the same collision prevention and safety features

## Features Implemented

✅ **Road Intersection Rendering**: Procedural cross-intersection with lane markings
✅ **Hybrid Spawning System**: Both manual (arrow keys) and automatic (R key toggle) vehicle spawning
✅ **Keyboard-Controlled Vehicle Spawning**: Manual spawning system using arrow keys
✅ **Automatic Spawning Toggle**: R key toggles automatic spawning every 1 second with console feedback
✅ **Fixed Lane Positioning**: Keyboard spawning uses deterministic lane selection for consistent behavior
✅ **Random Direction/Lane Selection**: Automatic spawning uses random directions and lanes
✅ **Car Sprite Rendering**: Uses actual car images from assets/ directory (Car.png, Audi.png, Mini_van.png, taxi.png, Police.png)
✅ **Random Car Sprite Selection**: Each spawned vehicle gets a randomly selected car sprite for visual variety
✅ **Larger Vehicle Size**: Vehicles rendered at 80x60 pixels for excellent visibility
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
- Add optional automatic spawning toggle (to switch between manual and automatic modes)
