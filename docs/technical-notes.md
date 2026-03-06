# Technical notes

Implementation details and additional run options for [SmartRoad: Traffic Intersection Simulation](../README.md). For installation and gameplay, start with the README.

## Run without SDL

With Rust and Cargo installed, the same simulation can run without graphics:

```bash
cargo run --locked --release --no-default-features -- --headless --duration 120 --seed 42 --drain
```

This runs automatic traffic as fast as the CPU allows. It simulates 120 seconds of spawn requests, then finishes all accepted vehicles. A seeded run with the same locked dependencies and build is reproducible.

## Scenarios and export

```bash
# Default automatic load: one request every 1.5 simulated seconds
cargo run --locked --release --no-default-features -- --headless --duration 120 --seed 42 --drain

# Saturation scenario: most requests will be rejected safely
cargo run --locked --release --no-default-features -- --headless --duration 120 --seed 7 --spawn-interval 0.05 --drain

# Spreadsheet-friendly final metrics
cargo run --locked --release --no-default-features -- --headless --duration 60 --seed 42 --csv results.csv

# All command-line options
cargo run --locked --no-default-features -- --help
```

Duration accepts 0.1–86,400 seconds; spawn interval accepts 0.05–60 seconds; seeds are unsigned 64-bit integers. Invalid arguments and failed file writes return an error. CSV export overwrites the specified file.

Without `--drain`, statistics include vehicles still on screen as active. With `--drain`, automatic spawning stops at the requested duration and traffic continues until every accepted vehicle exits; draining has a 600-second simulation limit. Safety violations or failure to drain return a nonzero exit status. Desktop `--duration` ends the run and opens the statistics dialog; `--drain` is headless-only. Add `--no-dialog` to print desktop statistics without a modal window, useful for unattended runs.

