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

How to build (Windows PowerShell):

```powershell
cd <project-root>
cargo build --release
cargo test
```

Next steps:
- Choose a rendering backend (ggez, bevy) and enable it in Cargo.toml
- Implement the simulation loop in `src/engine`
- Add asset files under `assets/` and load them via `src/assets`
