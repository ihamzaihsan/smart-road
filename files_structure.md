# Smart Road Project - File Structure Documentation

This document explains the purpose and organization of each file and folder in the Smart Road intersection management system.

## Project Overview
Smart Road is an autonomous vehicle intersection management system built with SDL2 and Rust. The project simulates intelligent traffic flow through intersections without traditional traffic lights, using advanced algorithms for vehicle coordination and safety.

## src structure
src/
├── bin/
│   └── visual_sim.rs                 # Main SDL2 application
│
├── core/                             # Core game logic
│   ├── mod.rs
│   ├── game.rs                       # Main game state (rename from game/mod.rs)
│   ├── intersection.rs               # Smart intersection algorithm
│   └── physics.rs                    # Vehicle physics (velocity, distance, time)
│
├── components/                       # Data structures
│   ├── mod.rs
│   ├── vehicle.rs                    # Vehicle struct & Direction enum
│   ├── intersection.rs               # Intersection state
│   └── route.rs                      # Route logic (straight, left, right)
│
├── systems/                          # Business logic systems
│   ├── mod.rs
│   ├── spawn_system.rs               # Vehicle spawning logic
│   ├── movement_system.rs            # Vehicle movement & physics
│   ├── collision_system.rs           # Safety distance & collision detection
│   ├── intersection_system.rs        # Smart intersection management
│   └── cleanup_system.rs             # Vehicle removal
│
├── rendering/                        # SDL2 graphics
│   ├── mod.rs
│   ├── renderer.rs                   # Main renderer
│   ├── road_renderer.rs              # Road drawing
│   ├── vehicle_renderer.rs           # Vehicle animation & rotation
│   └── ui_renderer.rs                # HUD and statistics display
│
├── input/                            # Input handling
│   ├── mod.rs
│   ├── keyboard.rs                   # Arrow keys, R key, ESC
│   └── commands.rs                   # Command processing
│
├── resources/                        # Game resources & config
│   ├── mod.rs
│   ├── config.rs                     # Global configuration
│   ├── assets.rs                     # Vehicle textures & sprites
│   ├── stats.rs                      # Statistics tracking
│   └── intersection_data.rs          # Intersection state data
│
├── algorithms/                       # Smart intersection algorithms
│   ├── mod.rs
│   ├── scheduler.rs                  # Intersection scheduling algorithm
│   ├── safety.rs                     # Safety distance calculations
│   └── optimization.rs               # Traffic flow optimization
│
├── animation/                        # Vehicle animation system
│   ├── mod.rs
│   ├── vehicle_animator.rs           # Vehicle movement animation
│   ├── turning_animator.rs           # Vehicle turning animation
│   └── sprite_manager.rs             # Sprite rotation & positioning
│
├── statistics/                       # Statistics & analytics
│   ├── mod.rs
│   ├── collector.rs                  # Data collection
│   ├── calculator.rs                 # Min/max/average calculations
│   ├── reporter.rs                   # Statistics window display
│   └── close_calls.rs                # Safety violation tracking
│
├── utils/                           # Utility functions
│   ├── mod.rs
│   ├── geometry.rs                   # Geometric calculations
│   ├── timing.rs                     # Time tracking utilities
│   └── math.rs                       # Mathematical helpers
│
├── tests/                           # Test files
│   ├── mod.rs
│   ├── intersection_tests.rs         # Intersection algorithm tests
│   ├── physics_tests.rs              # Physics system tests
│   └── integration_tests.rs          # End-to-end tests
│
├── lib.rs                           # Library root
└── main.rs                          # Alternative entry point (optional)

## Root Directory Structure

### Configuration Files
- **`Cargo.toml`** - Project dependencies and metadata for Rust package manager
- **`Cargo.lock`** - Locked dependency versions for reproducible builds
- **`README.md`** - Project overview and setup instructions
- **`projectInfo.md`** - Detailed project requirements and specifications
- **`todo.md`** - Development tasks and feature roadmap

### Assets Directory (`assets/`)
Contains all visual resources for the simulation:
- **Vehicle Images** - PNG files for different vehicle types (Ambulance.png, Audi.png, Black_viper.png, Car.png, Mini_truck.png, Mini_van.png, Police.png, taxi.png, truck.png)
- **`vehicles/car_base.svg`** - Base vector graphics for vehicle rendering

### Build Output (`target/`)
- Contains compiled binaries, dependencies, and build artifacts
- Generated automatically by Cargo build system
- **`debug/visual_sim.exe`** - Main executable for the simulation

## Source Code Structure (`src/`)

### Main Library Entry Point
- **`lib.rs`** - Primary library interface, exposes all public modules

### Executable Binary
- **`bin/visual_sim.rs`** - Main executable entry point for the SDL2 visual simulator

### Core System (`src/core/`)
Contains the fundamental game logic and systems:

- **`mod.rs`** - Module declarations and public interface
- **`game.rs`** - Main game state management, update loop, and coordinate system
- **`intersection.rs`** - Smart intersection management algorithms (core logic)
- **`physics.rs`** - Vehicle physics calculations, velocity, and safety distance management

### Component System (`src/components/`)
Entity-Component-System (ECS) data structures:

- **`mod.rs`** - Component module interface
- **`vehicle.rs`** - Vehicle entity definition, direction handling, and rotation logic

### Resource Management (`src/resources/`)
Global configuration and asset management:

- **`mod.rs`** - Resource module interface
- **`config.rs`** - Global simulation configuration (road, physics, simulation settings)
- **`assets.rs`** - Vehicle sprite loading and texture management for SDL2
- **`road_config.rs`** - Legacy road configuration (moved to config.rs)
- **`spawn_manager.rs`** - Legacy spawn management (moved to systems)
- **`stats.rs`** - Legacy statistics (moved to statistics module)

### System Modules (`src/systems/`)
ECS systems that operate on components:

- **`mod.rs`** - System module interface and public exports
- **`spawn_system.rs`** - Vehicle spawning logic with collision detection
- **`movement_system.rs`** - Vehicle movement and physics updates
- **`collision_system.rs`** - Collision detection and safety distance maintenance
- **`intersection_system.rs`** - Intersection management and traffic coordination
- **`cleanup_system.rs`** - Vehicle lifecycle management and cleanup
- **Legacy Files**:
  - **`vehicles.rs`** - Legacy vehicle systems (functionality moved to dedicated files)
  - **`scheduler.rs`** - Legacy scheduler (moved to algorithms)
  - **`cleanup.rs`** - Legacy cleanup (moved to cleanup_system.rs)

### Algorithm Library (`src/algorithms/`)
Smart intersection and traffic management algorithms:

- **`mod.rs`** - Algorithm module interface
- **`scheduler.rs`** - Core intersection scheduling algorithms for autonomous vehicles
- **`safety.rs`** - Safety distance calculations and collision avoidance
- **`optimization.rs`** - Traffic flow optimization and efficiency algorithms
- **`pathfinding.rs`** - Route planning and navigation algorithms

### Statistics and Analytics (`src/statistics/`)
Data collection and analysis for simulation evaluation:

- **`mod.rs`** - Statistics module interface
- **`collector.rs`** - Runtime data collection (speeds, passage times, etc.)
- **`calculator.rs`** - Statistical calculations (averages, rates, efficiency)
- **`reporter.rs`** - Report generation and data export
- **`close_calls.rs`** - Safety violation detection and tracking

### Rendering System (`src/rendering/`)
SDL2-based visual output:

- **`mod.rs`** - Complete SDL2 rendering system for roads, vehicles, and lane markings

### Input Handling (`src/input/`)
User interaction and control systems:

- **`mod.rs`** - Keyboard input processing for manual vehicle spawning and simulation control

### Animation System (`src/animation/`)
Smooth transitions and visual effects:

- **`mod.rs`** - Animation utilities with easing functions for smooth visual transitions

### Utility Libraries (`src/utils/`)
Common mathematical and geometric functions:

- **`mod.rs`** - Utility module interface
- **`geometry.rs`** - 2D geometry functions (points, distances, angles, rotations)
- **`math.rs`** - Mathematical utilities (trigonometry, interpolation, moving averages)

### Legacy Utilities (`src/util/`)
- **`mod.rs`** - Legacy utility interface
- **`geom.rs`** - Legacy geometry functions (kept for compatibility)

### Test Files (`src/tests/`)
- **`scheduler_tests.rs`** - Unit tests for scheduling algorithms

## Key Features

### Road Configuration
- **3 lanes per direction** (configurable)
- **Visual lane separators** with proper intersection gaps
- **Customizable lane width and intersection size**

### Vehicle Management
- **Multiple vehicle types** with different sprites
- **Realistic physics** with speed and acceleration
- **Collision detection** and safety distance maintenance
- **Manual and automatic spawning** controls

### Smart Intersection
- **Traffic light-free coordination** using intelligent algorithms
- **Real-time conflict resolution** between vehicles
- **Optimized traffic flow** for maximum efficiency

### Statistics and Monitoring
- **Real-time performance metrics** (speed, throughput, safety)
- **Close call detection** for safety analysis
- **Data export capabilities** for research analysis

### Visual Simulation
- **SDL2-based rendering** with smooth graphics
- **Real-time vehicle animation** with proper rotations
- **Interactive controls** for testing and demonstration

## Technology Stack
- **Language**: Rust (safe systems programming)
- **Graphics**: SDL2 (cross-platform multimedia library)
- **Math**: nalgebra (linear algebra and vector math)
- **Build System**: Cargo (Rust package manager)

## Usage
- **Build**: `cargo build`
- **Run Simulation**: `cargo run --bin visual_sim`
- **Controls**: Arrow keys for manual spawning, R key for auto-spawn toggle, ESC to quit

This modular architecture enables easy extension with new algorithms, vehicle types, and analysis tools while maintaining clear separation of concerns and robust performance.
