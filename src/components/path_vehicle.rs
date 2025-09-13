use crate::components::{Direction, Route, VehicleState};
use nalgebra::Vector2;

/// Enhanced vehicle structure with path-based navigation
#[derive(Debug, Clone)]
pub struct PathVehicle {
    pub id: u64,
    pub position: Vector2<f32>,
    pub width: f32,
    pub length: f32,
    pub speed: f32,
    pub target_speed: f32,
    pub rotation: f32,
    pub texture_index: usize,
    
    // Path-based navigation
    pub path_s: f32,              // Current position along path (arc-length parameter)
    pub path: crate::systems::conflict_system::Path,
    pub kinematics: crate::systems::conflict_system::Kinematics,
    
    // Route information
    pub approach: Direction,
    pub route: Route,
    pub state: VehicleState,
    
    // Conflict management
    pub next_conflict_cell: Option<crate::systems::conflict_system::ConflictCellId>,
    pub reserved_time_window: Option<(f32, f32)>, // (t_in, t_out)
    pub target_eta: Option<f32>,
}

impl PathVehicle {
    pub fn new(
        id: u64,
        start_position: Vector2<f32>,
        approach: Direction,
        route: Route,
        path: crate::systems::conflict_system::Path,
        texture_index: usize,
    ) -> Self {
        let mut kinematics = crate::systems::conflict_system::Kinematics::default();
        kinematics.v = 150.0; // Starting speed
        
        Self {
            id,
            position: start_position,
            width: 25.0,
            length: 50.0,
            speed: 150.0,
            target_speed: 200.0,
            rotation: approach.rotation_angle(),
            texture_index,
            path_s: 0.0,
            path,
            kinematics,
            approach,
            route,
            state: VehicleState::Moving,
            next_conflict_cell: None,
            reserved_time_window: None,
            target_eta: None,
        }
    }

    /// Update vehicle position based on path parameter
    pub fn update_from_path(&mut self, delta_time: f32) {
        // Update path parameter based on speed
        self.path_s += self.speed * delta_time;
        self.path_s = self.path_s.min(self.path.s_total);
        
        // Get position and heading from path
        let (new_position, heading) = self.path.pose_at_s(self.path_s);
        self.position = new_position;
        self.rotation = heading.y.atan2(heading.x);
        
        // Update kinematics
        self.kinematics.v = self.speed;
    }

    /// Calculate ETA to a specific path parameter
    pub fn eta_to_s(&self, s_target: f32) -> f32 {
        self.kinematics.eta_to_s(self.path_s, s_target)
    }

    /// Check if vehicle has completed its path
    pub fn is_path_complete(&self) -> bool {
        self.path_s >= self.path.s_total - 1.0
    }

    /// Get current progress along path (0.0 to 1.0)
    pub fn path_progress(&self) -> f32 {
        if self.path.s_total > 0.0 {
            (self.path_s / self.path.s_total).clamp(0.0, 1.0)
        } else {
            1.0
        }
    }

    /// Apply speed control with hysteresis
    pub fn apply_speed_control(&mut self, target_eta: f32, current_time: f32) {
        if let Some(window) = self.reserved_time_window {
            let eta_error = window.0 - current_time - target_eta;
            
            // Hysteresis bands for speed changes
            let current_speed_idx = self.get_current_speed_index();
            
            if eta_error > 0.6 {
                // Too early, slow down
                self.set_speed_level(0); // slow
            } else if eta_error > 0.2 && current_speed_idx > 0 {
                // Slightly early, medium speed
                self.set_speed_level(1); // med
            } else if eta_error < -0.3 {
                // Too late, speed up
                self.set_speed_level(2); // fast
            }
            // else keep current speed (hysteresis)
        }
    }

    fn get_current_speed_index(&self) -> usize {
        let speeds = &self.kinematics.v_levels;
        for (i, &speed) in speeds.iter().enumerate() {
            if (self.target_speed - speed).abs() < 10.0 {
                return i;
            }
        }
        1 // default to medium
    }

    fn set_speed_level(&mut self, level: usize) {
        if level < self.kinematics.v_levels.len() {
            self.target_speed = self.kinematics.v_levels[level];
        }
    }

    /// Update speed towards target with acceleration limits
    pub fn update_speed(&mut self, delta_time: f32) {
        let speed_diff = self.target_speed - self.speed;
        let max_accel = if speed_diff > 0.0 {
            self.kinematics.a_max
        } else {
            -self.kinematics.d_max
        };
        
        let speed_change = max_accel * delta_time;
        if speed_diff.abs() < speed_change.abs() {
            self.speed = self.target_speed;
        } else {
            self.speed += speed_change.copysign(speed_diff);
        }
        
        self.speed = self.speed.max(0.0);
    }
}
