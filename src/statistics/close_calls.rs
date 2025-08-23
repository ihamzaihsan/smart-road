use crate::components::Vehicle;

pub struct CloseCallDetector;

impl CloseCallDetector {
    
    pub fn detect_close_calls(vehicles: &[Vehicle], min_distance: f32) -> Vec<(usize, usize)> {
        let mut close_calls = Vec::new();
        
        for i in 0..vehicles.len() {
            for j in (i + 1)..vehicles.len() {
                let distance = (vehicles[i].position - vehicles[j].position).magnitude();
                if distance < min_distance {
                    close_calls.push((i, j));
                }
            }
        }
        
        close_calls
    }

    pub fn count_close_calls(vehicles: &[Vehicle], min_distance: f32) -> u32 {
        Self::detect_close_calls(vehicles, min_distance).len() as u32
    }
}