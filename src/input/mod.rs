use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::collections::HashSet;

/// Input system for handling keyboard events
#[derive(Debug)]
pub struct InputSystem {
    /// Keys that are currently pressed
    pressed_keys: HashSet<Keycode>,
    /// Keys that were just pressed this frame
    just_pressed_keys: HashSet<Keycode>,
    /// Keys that were just released this frame
    just_released_keys: HashSet<Keycode>,
}

impl InputSystem {
    /// Create a new input system
    pub fn new() -> Self {
        Self {
            pressed_keys: HashSet::new(),
            just_pressed_keys: HashSet::new(),
            just_released_keys: HashSet::new(),
        }
    }

    /// Update the input system with SDL2 events
    pub fn update(&mut self, events: &[Event]) {
        // Clear just pressed/released keys from previous frame
        self.just_pressed_keys.clear();
        self.just_released_keys.clear();

        // Process events
        for event in events {
            match event {
                Event::KeyDown { keycode: Some(keycode), repeat: false, .. } => {
                    if !self.pressed_keys.contains(keycode) {
                        self.just_pressed_keys.insert(*keycode);
                        self.pressed_keys.insert(*keycode);
                    }
                }
                Event::KeyUp { keycode: Some(keycode), .. } => {
                    if self.pressed_keys.contains(keycode) {
                        self.just_released_keys.insert(*keycode);
                        self.pressed_keys.remove(keycode);
                    }
                }
                _ => {}
            }
        }
    }

    /// Check if a key is currently pressed
    pub fn is_pressed(&self, keycode: Keycode) -> bool {
        self.pressed_keys.contains(&keycode)
    }

    /// Check if a key was just pressed this frame
    pub fn just_pressed(&self, keycode: Keycode) -> bool {
        self.just_pressed_keys.contains(&keycode)
    }

    /// Check if a key was just released this frame
    pub fn just_released(&self, keycode: Keycode) -> bool {
        self.just_released_keys.contains(&keycode)
    }

    /// Get all currently pressed keys
    pub fn get_pressed_keys(&self) -> &HashSet<Keycode> {
        &self.pressed_keys
    }

    /// Get all keys that were just pressed this frame
    pub fn get_just_pressed_keys(&self) -> &HashSet<Keycode> {
        &self.just_pressed_keys
    }

    /// Get all keys that were just released this frame
    pub fn get_just_released_keys(&self) -> &HashSet<Keycode> {
        &self.just_released_keys
    }
}

impl Default for InputSystem {
    fn default() -> Self {
        Self::new()
    }
}
