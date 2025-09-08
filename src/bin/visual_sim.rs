use sdl2::event::Event;
use std::time::Duration;
use smart_road::core::Game;
use smart_road::rendering::Renderer;
fn main() -> Result<(), String> {
    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;
    let _image_context = sdl2::image::init(sdl2::image::InitFlag::PNG)?;
    let window = video_subsystem
        .window("Smart Road Simulator", 1200, 800)
        .position_centered()
        .build()
        .map_err(|e| e.to_string())?;
    let canvas = window
        .into_canvas()
        .accelerated()
        .build()
        .map_err(|e| e.to_string())?;
    let texture_creator = canvas.texture_creator();
    let mut renderer = Renderer::new(canvas, &texture_creator);
    let mut game = Game::new(&texture_creator)?;
    let mut event_pump = sdl_context.event_pump()?;
    println!("Smart Road Simulator - SDL2 Version");
    println!("Controls:");
    println!("  Arrow Keys: Manual vehicle spawning");
    println!("  R Key: Toggle automatic spawning");
    println!("  ESC or close window: Quit");
    'running: loop {
        let events: Vec<Event> = event_pump.poll_iter().collect();
        if !game.update(&events) {
            break 'running;
        }
        for event in &events {
            match event {
                Event::Quit { .. } |
                Event::KeyDown { keycode: Some(sdl2::keyboard::Keycode::Escape), .. } => {
                    break 'running;
                }
                _ => {}
            }
        }
        if let Err(e) = game.render(&mut renderer) {
            eprintln!("Render error: {}", e);
            return Err(e);
        }
        std::thread::sleep(Duration::from_millis(16));
    }
    println!("Smart Road Simulator shutting down...");
    Ok(())
}
