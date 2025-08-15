# Smart Road — Project TODO

This file lists actionable milestones, priorities, and quick next steps to finish the simulation.

## Status legend
- [ ] = To do
- [~] = In progress
- [x] = Done

## Milestones

- [x] Draw roads (procedural road & lane markings)
	- Implemented in `src/systems/roads.rs` and launched by `src/bin/visual_sim.rs`.
- [x] Wire `RoadConfig` into road drawing
	- `RoadConfig` inserted in `visual_sim` and consumed by `setup_roads`.

- [ ] Vehicles: spawn, assets, and basic movement
	- Spawn vehicles using keyboard controls (arrow keys + R for continuous spawn).
	- Load vehicle sprites from `assets/vehicles/` and attach `Vehicle` component (`src/components/vehicle.rs`).
	- Implement `move_vehicles` to update `Transform` and orientation while animating turns.

- [ ] Vehicle physics & safety
	- Provide discrete velocity states (at least 3 speeds) per vehicle.
	- Enforce a safe distance; detect close-calls and avoid collisions.
	- Record per-vehicle time/distance/velocity for stats.

- [ ] Scheduler / intersection control (core algorithm)
	- Implement reservation-based or priority scheduler in `src/systems/scheduler.rs` (pure logic, covered by unit tests in `src/tests/`).
	- Integrate scheduler with vehicle movement (speed gating, reservations).

- [ ] Controls & user interaction
	- Keyboard: Arrow keys to spawn per-direction, `R` toggle for continuous random spawning, `Esc` to finish and show stats window.
	- Prevent stacking spawns on top of each other when keys are spammed.

- [ ] HUD & Stats window
	- Implement `src/ui/hud.rs` for on-screen counters and debug toggles.
	- Implement `src/ui/stats_window.rs` to show final statistics when simulation ends.

- [ ] Tests & verification
	- Unit tests for `util/geom.rs` (already present) and scheduler logic in `src/tests/scheduler_tests.rs`.
	- Add tests for reservation conflicts and safety distance.

- [ ] Performance & rendering improvements
	- Replace many dashed-sprite entities with a batched mesh or single mesh draw call if needed.
	- Consider texture atlas for vehicle sprites.

- [ ] Polish
	- Camera follow and zoom controls.
	- Smooth vehicle turn animations and sprite orientation.
	- Logging, CLI flags or config file for scenario presets.

## Quick run (dev)
From project root:
```powershell
cargo run --bin visual_sim
```

## Next immediate actions (priority)
1. Implement vehicle spawn + sprites so you can see motion on top of the drawn roads.
2. Implement simple scheduler (reservation or passage ordering) as a pure function with unit tests.
3. Add HUD counters for live debugging (vehicle count, average speed).

If you want, I can start on (1) now: spawn vehicles with placeholder sprites and simple forward motion. Which task should I pick next?
