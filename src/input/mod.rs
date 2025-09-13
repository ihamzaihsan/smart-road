use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::collections::HashSet;

#[derive(Debug)]
pub struct InputSystem {
    
    pressed_keys: HashSet<Keycode>,
    
    just_pressed_keys: HashSet<Keycode>,
    
    just_released_keys: HashSet<Keycode>,
}

impl InputSystem {
    
    pub fn new() -> Self {
        Self {
            pressed_keys: HashSet::new(),
            just_pressed_keys: HashSet::new(),
            just_released_keys: HashSet::new(),
        }
    }

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

    pub fn is_pressed(&self, keycode: Keycode) -> bool {
        self.pressed_keys.contains(&keycode)
    }

    pub fn just_pressed(&self, keycode: Keycode) -> bool {
        self.just_pressed_keys.contains(&keycode)
    }

    pub fn just_released(&self, keycode: Keycode) -> bool {
        self.just_released_keys.contains(&keycode)
    }

    pub fn get_pressed_keys(&self) -> &HashSet<Keycode> {
        &self.pressed_keys
    }

    pub fn get_just_pressed_keys(&self) -> &HashSet<Keycode> {
        &self.just_pressed_keys
    }

    pub fn get_just_released_keys(&self) -> &HashSet<Keycode> {
        &self.just_released_keys
    }
}

impl Default for InputSystem {
    fn default() -> Self {
        Self::new()
    }
}