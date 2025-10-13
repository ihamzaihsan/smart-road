# Smart Road Intersection Simulator (SDL2)

A traffic intersection simulator built with Rust and SDL2, featuring hybrid vehicle spawning (manual + automatic), collision prevention, and realistic vehicle movement.

## Overview

This project simulates a 4-way intersection with both manual keyboard-controlled and automatic vehicle spawning. Vehicles are rendered using actual car sprites and move through the intersection with realistic physics and collision prevention.

## Prerequisites

### SDL2 Installation

This project requires SDL2 to be installed on your system:

#### Windows
1. Download SDL2 development libraries from [libsdl.org](https://www.libsdl.org/download-2.0.php)
2. Extract to a folder (e.g., `C:\SDL2`)
3. Add the `lib\x64` directory to your PATH environment variable
4. Set environment variable: `SDL2_LIB_DIR=C:\SDL2\lib\x64`

#### Linux (Ubuntu/Debian)
```bash
sudo apt-get install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev
```

#### macOS
```bash
brew install sdl2 sdl2_image sdl2_ttf
```

## Quick Start

```bash
# Clone the repository
git clone <repository-url>
cd smart-road

# Run the visual simulation
cargo run --bin visual_sim
```

The simulation will open a window showing the intersection. Use keyboard controls to spawn vehicles manually or toggle automatic spawning.

## Priority-Based Intersection Management

This project implements an advanced **priority-based intersection entry system** that prevents vehicular collisions through proactive traffic management:

### Key Features
- **🚦 Priority Queue System**: Vehicles wait at lane separators based on route priority
- **🚗 Route Priorities**: Right turns (highest) → Straight → Left turns (lowest)  
- **🛑 Collision Prevention**: Only one vehicle can access intersection at a time
- **📊 Visual Feedback**: Color-coded indicators show vehicle states and priorities
- **⚡ FIFO Processing**: Fair ordering within same priority levels

### Priority System Benefits
- **Zero Collisions**: Prevents conflicts by controlling intersection entry
- **Optimized Flow**: Priority-based processing improves traffic efficiency  
- **Safe Operations**: Robust queue management with deadlock prevention
- **Visual Clarity**: Real-time status indicators for system monitoring

### Testing the Priority System
```bash
# Run priority system demo
cargo run --bin priority_demo

# Run simple priority test
cargo run --bin simple_priority_test
```

## Project Structure

- `src/bin/visual_sim.rs` - Main SDL2 application with game loop
- `src/bin/priority_demo.rs` - Priority system demonstration
- `src/components/` - Data structures (Vehicle, Direction, etc.)
- `src/systems/` - Traffic management systems including priority control
- `src/resources/` - Game resources (SpawnManager, RoadConfig, etc.)
- `src/rendering/` - SDL2 rendering system for roads and vehicles
- `src/input/` - SDL2 input handling system
- `src/game/` - Main game logic and state management
- `assets/` - Car sprite images (PNG format)

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
