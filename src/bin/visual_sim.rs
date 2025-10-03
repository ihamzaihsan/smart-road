use sdl2::event::Event;
use std::time::Duration;

use smart_road::core::Game;
use smart_road::rendering::Renderer;
use smart_road::resources::StatsSummary;

fn display_stats(summary: StatsSummary) {
    let format_value = |value: Option<f32>| -> String {
        value
            .map(|v| format!("{:.2}", v))
            .unwrap_or_else(|| "N/A".to_string())
    };

    let stats_message = format!(
        "Simulation complete!\n\nTotal vehicles passed: {}\nMax speed: {}\nMin speed: {}\nLongest travel time: {} seconds\nShortest travel time: {} seconds\nClose calls: {}\nCollisions: {}",
        summary.total_vehicles_passed,
        format_value(summary.max_speed),
        format_value(summary.min_speed),
        format_value(summary.max_travel_time),
        format_value(summary.min_travel_time),
        summary.close_calls,
        summary.collisions,
    );

    let _ = sdl2::messagebox::show_simple_message_box(
        sdl2::messagebox::MessageBoxFlag::INFORMATION,
        "Smart Intersection Statistics",
        &stats_message,
        None,
    );
}

fn main() -> Result<(), String> {
    // Initialize SDL2
    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;
    let _image_context = sdl2::image::init(sdl2::image::InitFlag::PNG)?;

    // Create window
    let window = video_subsystem
        .window("Smart Road Simulator", 1200, 800)
        .position_centered()
        .build()
        .map_err(|e| e.to_string())?;

    // Create canvas
    let canvas = window
        .into_canvas()
        .accelerated()
        .build()
        .map_err(|e| e.to_string())?;

    // Create texture creator
    let texture_creator = canvas.texture_creator();

    // Create renderer
    let mut renderer = Renderer::new(canvas, &texture_creator);

    // Create game instance
    let mut game = Game::new(&texture_creator)?;

    // Event pump for handling events
    let mut event_pump = sdl_context.event_pump()?;

    println!("Smart Road Simulator - SDL2 Version");
    println!("Controls:");
    println!("  Arrow Keys: Manual vehicle spawning");
    println!("  R Key: Toggle automatic spawning");
    println!("  ESC or close window: Quit");

    // Main game loop
    'running: loop {
        // Collect events
        let events: Vec<Event> = event_pump.poll_iter().collect();

        // Update game logic
        if !game.update(&events) {
            break 'running;
        }

        if let Some(summary) = game.take_stats_summary() {
            display_stats(summary);
        }

        // Render
        if let Err(e) = game.render(&mut renderer) {
            eprintln!("Render error: {}", e);
            return Err(e);
        }

        // Cap frame rate to ~60 FPS
        std::thread::sleep(Duration::from_millis(16));
    }

    println!("Smart Road Simulator shutting down...");

    if let Some(summary) = game.take_stats_summary() {
        display_stats(summary);
    }

    Ok(())
}
