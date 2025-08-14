# Smart Road — Rust implementation plan

## Quick project summary

Create a smart intersection simulator where only autonomous vehicles (AVs) traverse a cross intersection without traffic lights. The system must control vehicle speeds (at least 3 velocity levels), enforce a positive safety distance, animate vehicles turning/right/straight/left, accept keyboard input to spawn vehicles and toggle continuous random generation, and produce exit statistics (max/min velocities & times, max passed vehicles, and close calls).

## What you should learn (Rust-focused)

- Rust basics: ownership, borrowing, lifetimes, enums, structs, traits, error handling (Result/Option).
- Cargo workflow: create/build/test, cargo fmt, clippy, workspaces.
- Concurrency & async: threads, channels (std::sync::mpsc), or async (tokio) if needed for decoupling input/render/simulation.
- 2D graphics/game loop: choose a crate (recommended: `macroquad` or `ggez` for simplicity; `bevy` is more powerful but heavier). Learn event handling and drawing in the chosen crate.
- Linear algebra & geometry: vectors, rotation, transforms (2D), simple collision detection and distance math.
- Game loop & fixed timestep simulation: integrate velocity/position reliably; separate update and render.
- Basic physics: constant velocity, acceleration, distance = velocity * time, time tracking per vehicle.
- Project tooling: unit tests, integration tests, simple CI, and asset loading (images/sprites).

## Requirements checklist (extracted from spec)

1. Vehicles with fixed route per lane (r/s/l) — Planned
2. Vehicles are autonomous; control velocity via system (>=3 discrete speeds) — Planned
3. Safety distance > 0 and collision avoidance — Planned
4. Each vehicle tracks time, distance, velocity (and time to leave intersection) — Planned
5. Animation with turning (sprite rotation/transform while moving) — Planned
6. Keyboard commands: arrow keys spawn vehicles, R toggles continuous random spawn, Esc ends sim and shows stats — Planned
7. No overlapping spawn when keys spammed — Planned
8. Statistics on exit: max/min velocity, max/min time, max vehicles passed, close calls — Planned

## High-level architecture

- bin/main.rs — entry, CLI flags, choose backend (window size, assets path).
- src/app.rs — main app struct: initialization, game loop, state (running, paused), input handling.
- src/simulation.rs — Simulation struct: time step, spawn rules, lane queues.
- src/vehicle.rs — Vehicle struct and methods (update physics, render transform, enter/exit times).
- src/intersection.rs — Intersection manager: safe reservations, conflict resolution, speed advisories.
- src/renderer.rs — Rendering wrapper around chosen graphics crate; sprite loading, coordinate mapping.
- src/input.rs — Keyboard handlers and spawn logic (debouncing spawn positions).
- src/stats.rs — Collects and computes statistics; export on exit.
- assets/ — sprites, road images, optional JSON metadata for sprites.
- tests/ — unit tests for physics, safety distance, reservation logic.

Data contract (short):
- Vehicle { id: u64, lane: Lane, route: Route, position: Vec2, velocity: f32, target_velocity: f32, length: f32, time_entered: Option<Instant>, time_exited: Option<Instant>, distance_travelled: f32 }
- Simulation update(dt): updates vehicles, enforces safety distance, asks IntersectionManager for allowed speed or reservation.

## Intersection control algorithm (initial approach)

Start simple and safe: central scheduler with discrete timesteps.

- Per-lane FIFO queue: vehicles maintain a desired speed level (low/medium/high). Intersection manager assigns a target velocity to each active vehicle such that predicted trajectories do not collide and safety distance is preserved.
- Prediction window: for each vehicle, simulate short horizon positions; if two vehicles from different lanes have a conflict in the intersection region at the same time, postpone one by lowering its target speed or stopping before intersection.
- Close calls metric: count occurrences where predicted distance during crossing < safety threshold but no collision occurred.

Notes: After MVP, consider an advanced reservation table algorithm (time-space reservations) for higher throughput.

## Edge cases to handle

- Multiple vehicles in same lane (respect safety distance and prevent spawn overlap).
- Simultaneous conflicting turns (left vs straight) must be resolved.
- Very slow vehicles blocking others — provide timeout policies.
- Rounding/time-stepping artifacts — use fixed timestep and sub-stepping if needed.

## Milestones & tasks

Milestone 1 — Project setup (1 day)
- Initialize Cargo project and git repo.
- Choose graphics crate and add dependencies.
- Create assets folder and a minimal README.

Milestone 2 — Core simulation + vehicles (2–4 days)
- Implement Vehicle struct and physics update (position, velocity, distance, time counters).
- Implement spawn logic per lane, ensure debounced spawns.
- Implement safety distance enforcement inside a lane.
- Unit tests for physics and spacing.

Milestone 3 — Intersection manager (2–3 days)
- Implement central scheduler that prevents collisions (initially conservative: stop before intersection to wait).
- Implement simple prediction to allow speed adjustments instead of full stops.
- Tests for conflict scenarios.

Milestone 4 — Rendering & animation (2–4 days)
- Integrate rendering crate; draw road and vehicles.
- Implement sprite rotation/turn animation for r/s/l.
- Add keyboard handlers for Arrow keys, R, Esc.

Milestone 5 — Statistics, polish, and packaging (1–2 days)
- Collect runtime statistics and show them on exit (GUI modal or console window).
- Tidy UI, add simple HUD for current vehicles, toggle debug overlays (bounds and reservation windows).
- Run lints, add CI config (optional).

## Tests and quality gates

- cargo build — ensure compilation.
- cargo clippy — linting.
- cargo test — unit tests for physics and intersection logic.
- Manual smoke test: run app, spawn vehicles, press Esc and verify stats.

## Tools & crates to consider

- macroquad — simple, single-file-style game framework (easy to prototype).
- ggez — game framework with good docs (Rust 2D games).
- bevy — ECS, more features, steeper learning curve (good for larger scope).
- glam — small math crate (Vec2/Mat2) used by many game crates.

## Next immediate steps (for me to do now if you want)

1. Create the Cargo project skeleton and choose the rendering crate.
2. Implement a minimal simulation loop with a single vehicle moving across the intersection (smoke test).

---

If you want, I can initialize the Cargo project and scaffold the file layout now (I recommend using `macroquad` for a fast prototype). Tell me if you want me to proceed with scaffolding, and which graphics crate you prefer.

