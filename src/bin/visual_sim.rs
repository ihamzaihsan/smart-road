use smart_road::core::Simulation;
use smart_road::resources::{StatsSummary, FIXED_STEP};
use smart_road::PROJECT_NAME;
use std::path::PathBuf;

struct Options {
    headless: bool,
    demo: bool,
    drain: bool,
    no_dialog: bool,
    duration: Option<f64>,
    seed: u64,
    spawn_interval: f64,
    csv: Option<PathBuf>,
}

impl Options {
    fn parse() -> Result<Option<Self>, String> {
        let mut options = Self {
            headless: false,
            demo: false,
            drain: false,
            no_dialog: false,
            duration: None,
            seed: 42,
            spawn_interval: 1.5,
            csv: None,
        };
        let mut args = std::env::args().skip(1);
        while let Some(argument) = args.next() {
            match argument.as_str() {
                "--help" | "-h" => {
                    println!("{PROJECT_NAME}\n\nUsage: cargo run -- [OPTIONS]\n\n  --demo                  Start automatic traffic in the desktop app\n  --seed NUMBER           Reproducible random seed (default: 42)\n  --spawn-interval SEC    Automatic request interval: 0.05..60 (default: 1.5)\n  --help                  Show this help\n\nDesktop: arrows spawn from each approach; R toggles automatic traffic; Esc shows statistics.");
                    return Ok(None);
                }
                "--demo" => options.demo = true,
                "--seed" | "--spawn-interval" => {
                    let input = args
                        .next()
                        .ok_or_else(|| format!("missing value for {argument}"))?;
                    match argument.as_str() {
                        "--duration" => {
                            options.duration = Some(input.parse().map_err(|_| "invalid duration")?)
                        }
                        "--seed" => {
                            options.seed = input
                                .parse()
                                .map_err(|_| "seed must be an unsigned integer")?
                        }
                        "--spawn-interval" => {
                            options.spawn_interval =
                                input.parse().map_err(|_| "invalid spawn interval")?
                        }
                        _ => options.csv = Some(PathBuf::from(input)),
                    }
                }
                _ => return Err(format!("unknown option: {argument}; use --help")),
            }
        }
        if options
            .duration
            .is_some_and(|v| !v.is_finite() || !(0.1..=86400.0).contains(&v))
        {
            return Err("duration must be between 0.1 and 86400 seconds".into());
        }
        if options.drain && !options.headless {
            return Err("--drain requires --headless".into());
        }
        if !options.spawn_interval.is_finite() || !(0.05..=60.0).contains(&options.spawn_interval) {
            return Err("spawn interval must be between 0.05 and 60 seconds".into());
        }
        Ok(Some(options))
    }
}

fn run_headless(options: &Options) -> Result<StatsSummary, String> {
    let mut simulation = Simulation::new(options.seed, options.spawn_interval)?;
    simulation.toggle_automatic();
    let ticks = (options.duration.unwrap_or(60.0) / FIXED_STEP).ceil() as u64;
    for _ in 0..ticks {
        simulation.tick();
    }
    if options.drain {
        simulation.automatic = false;
        let deadline = simulation.time + 600.0;
        while !simulation.vehicles.is_empty() {
            if simulation.time >= deadline {
                return Err("traffic failed to drain within 600 simulated seconds".into());
            }
            simulation.tick();
        }
    }
    Ok(simulation.summary())
}

#[cfg(feature = "graphics")]
fn run_desktop(options: &Options) -> Result<StatsSummary, String> {
    use sdl2::{event::Event, keyboard::Keycode};
    use smart_road::{components::Direction, rendering::Renderer};
    use std::time::{Duration, Instant};

    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let window = video
        .window(PROJECT_NAME, 1200, 800)
        .position_centered()
        .resizable()
        .build()
        .map_err(|e| e.to_string())?;
    // SDL chooses a suitable renderer; software-only environments are supported.
    let mut canvas = window.into_canvas().build().map_err(|e| e.to_string())?;
    canvas
        .set_logical_size(1200, 800)
        .map_err(|e| e.to_string())?;
    let texture_creator = canvas.texture_creator();
    let mut renderer = Renderer::new(canvas, &texture_creator)?;
    let mut simulation = Simulation::new(options.seed, options.spawn_interval)?;
    if options.demo {
        simulation.toggle_automatic();
    }
    let mut event_pump = sdl.event_pump()?;
    let mut last_frame = Instant::now();
    let mut accumulator = 0.0;
    'running: loop {
        let frame_start = Instant::now();
        // Ignore long OS stalls; never integrate one giant movement step.
        accumulator += frame_start
            .duration_since(last_frame)
            .as_secs_f64()
            .min(0.25);
        last_frame = frame_start;
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => break 'running,
                Event::KeyDown {
                    keycode: Some(key),
                    repeat: false,
                    ..
                } => match key {
                    Keycode::Up => {
                        simulation.spawn(Direction::North);
                    }
                    Keycode::Down => {
                        simulation.spawn(Direction::South);
                    }
                    Keycode::Right => {
                        simulation.spawn(Direction::East);
                    }
                    Keycode::Left => {
                        simulation.spawn(Direction::West);
                    }
                    Keycode::R => simulation.toggle_automatic(),
                    _ => {}
                },
                _ => {}
            }
        }
        while accumulator >= FIXED_STEP {
            simulation.tick();
            accumulator -= FIXED_STEP;
            if options
                .duration
                .is_some_and(|duration| simulation.time >= duration)
            {
                break 'running;
            }
        }
        renderer.render(&simulation)?;
        if let Some(remaining) =
            Duration::from_secs_f64(1.0 / 60.0).checked_sub(frame_start.elapsed())
        {
            std::thread::sleep(remaining);
        }
    }
    let summary = simulation.summary();
    renderer.render(&simulation)?;
    if !options.no_dialog {
        if let Err(error) = sdl2::messagebox::show_simple_message_box(
            sdl2::messagebox::MessageBoxFlag::INFORMATION,
            &format!("{PROJECT_NAME} - Statistics"),
            &summary.to_string(),
            Some(renderer.window()),
        ) {
            eprintln!(
                "Statistics dialog unavailable: {error}. Full statistics are printed to the terminal."
            );
        }
    }
    Ok(summary)
}

fn run() -> Result<(), String> {
    let Some(options) = Options::parse()? else {
        return Ok(());
    };
    let summary = if options.headless {
        run_headless(&options)?
    } else {
        #[cfg(feature = "graphics")]
        {
            run_desktop(&options)?
        }
        #[cfg(not(feature = "graphics"))]
        {
            return Err(
                "desktop support is disabled; use --headless or build with --features graphics"
                    .into(),
            );
        }
    };
    println!("{PROJECT_NAME}\nSeed: {}\n\n{summary}", options.seed);
    if let Some(path) = options.csv {
        std::fs::write(&path, summary.csv())
            .map_err(|e| format!("cannot write {}: {e}", path.display()))?;
    }
    if summary.collisions > 0 || summary.close_calls > 0 {
        return Err("the scenario recorded safety violations; see the statistics above".into());
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("SmartRoad: {error}");
        std::process::exit(1);
    }
}
