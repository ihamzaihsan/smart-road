use crate::{
    components::{Vehicle, path_vehicle::PathVehicle},
    core::physics::VehiclePhysics,
    resources::stats::Stats,
    systems::{
        conflict_system::{ConflictSystem},
        capsule_collision::{CapsuleCollision, TTCAnalysis},
    },
};
use nalgebra::Vector2;

/// Enhanced collision system with conflict-based detection
pub struct EnhancedCollisionSystem {
    pub conflict_system: ConflictSystem,
    pub intersection_bounds: (Vector2<f32>, Vector2<f32>), // (min, max) of intersection area
    pub capsule_checker: CapsuleCollision,
}

impl EnhancedCollisionSystem {
    pub fn new(intersection_center: Vector2<f32>, intersection_size: f32) -> Self {
        let half_size = intersection_size / 2.0;
        let bounds = (
            intersection_center - Vector2::new(half_size, half_size),
            intersection_center + Vector2::new(half_size, half_size),
        );
        
        Self {
            conflict_system: ConflictSystem::new(),
            intersection_bounds: bounds,
            capsule_checker: CapsuleCollision,
        }
    }

    /// Main collision detection with conflict resolution
    pub fn detect_and_resolve_conflicts(
        &mut self,
        vehicles: &mut [PathVehicle],
        current_time: f32,
        delta_time: f32,
        stats: &mut Stats,
    ) -> Vec<u64> {
        let mut collided_ids = Vec::new();
        
        // Clean up expired reservations
        self.conflict_system.cleanup_expired_reservations(current_time);
        
        // Update conflict management for each vehicle
        for vehicle in vehicles.iter_mut() {
            self.update_vehicle_conflicts(vehicle, current_time);
        }

        // Detect and resolve conflicts
        let conflicts = self.detect_conflicts(vehicles, current_time);
        self.resolve_conflicts(vehicles, &conflicts, current_time);

        // Continuous collision check in intersection area
        collided_ids.extend(self.check_intersection_collisions(vehicles, delta_time, stats));

        collided_ids
    }

    /// Update conflict cell management for a vehicle
    fn update_vehicle_conflicts(&mut self, vehicle: &mut PathVehicle, current_time: f32) {
        // Get conflict spans for this vehicle's route
        if let Some(spans) = self.conflict_system.get_conflict_spans(vehicle.approach, vehicle.route) {
            // Find the next conflict cell the vehicle will encounter
            for span in spans {
                if vehicle.path_s < span.s_in && vehicle.next_conflict_cell.is_none() {
                    vehicle.next_conflict_cell = Some(span.cell);
                    
                    // Try to reserve this cell
                    let eta = vehicle.eta_to_s(span.s_in);
                    let duration = (span.s_out - span.s_in) / vehicle.speed.max(1.0);
                    let separation = 0.5; // 500ms separation
                    
                    match self.conflict_system.reserve_cell(
                        span.cell,
                        vehicle.id,
                        current_time + eta,
                        duration,
                        separation,
                    ) {
                        Ok((t_in, t_out)) => {
                            vehicle.reserved_time_window = Some((t_in, t_out));
                            vehicle.target_eta = Some(eta);
                        }
                        Err(_) => {
                            // Reservation failed, need to adjust speed
                            vehicle.target_speed = vehicle.kinematics.v_levels[0]; // Slow down
                        }
                    }
                    break;
                }
                
                // Check if we've passed this conflict cell
                if vehicle.path_s > span.s_out && vehicle.next_conflict_cell == Some(span.cell) {
                    vehicle.next_conflict_cell = None;
                    vehicle.reserved_time_window = None;
                    vehicle.target_eta = None;
                }
            }
        }
    }

    /// Detect potential conflicts between vehicles
    fn detect_conflicts(&self, vehicles: &[PathVehicle], _current_time: f32) -> Vec<(usize, usize)> {
        let mut conflicts = Vec::new();
        
        for i in 0..vehicles.len() {
            for j in (i + 1)..vehicles.len() {
                let vehicle_a = &vehicles[i];
                let vehicle_b = &vehicles[j];
                
                // Check if vehicles are targeting the same conflict cell
                if let (Some(cell_a), Some(cell_b)) = (&vehicle_a.next_conflict_cell, &vehicle_b.next_conflict_cell) {
                    if cell_a == cell_b {
                        // Check if they're actually crossing (not following)
                        if self.capsule_checker.are_crossing(vehicle_a, vehicle_b) {
                            // Check TTC to conflict
                            if let (Some(spans_a), Some(spans_b)) = (
                                self.conflict_system.get_conflict_spans(vehicle_a.approach, vehicle_a.route),
                                self.conflict_system.get_conflict_spans(vehicle_b.approach, vehicle_b.route),
                            ) {
                                for span_a in spans_a {
                                    for span_b in spans_b {
                                        if TTCAnalysis::check_eta_conflict(
                                            vehicle_a,
                                            vehicle_b,
                                            span_a,
                                            span_b,
                                            0.4, // 400ms separation
                                        ) {
                                            conflicts.push((i, j));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        conflicts
    }

    /// Resolve conflicts using priority rules
    fn resolve_conflicts(&self, vehicles: &mut [PathVehicle], conflicts: &[(usize, usize)], _current_time: f32) {
        for &(i, j) in conflicts {
            let (vehicle_a, vehicle_b) = (&vehicles[i], &vehicles[j]);
            
            // Calculate ETAs to conflict
            let eta_a = vehicle_a.target_eta.unwrap_or(0.0);
            let eta_b = vehicle_b.target_eta.unwrap_or(0.0);
            
            // Determine which vehicle should yield
            let yielding_id = TTCAnalysis::determine_yielding_vehicle(vehicle_a, vehicle_b, eta_a, eta_b);
            
            // Apply yielding behavior
            if yielding_id == vehicle_a.id {
                let vehicle_a = &mut vehicles[i];
                vehicle_a.target_speed = vehicle_a.kinematics.v_levels[0]; // Slow speed
                vehicle_a.state = crate::components::VehicleState::Yielding;
            } else {
                let vehicle_b = &mut vehicles[j];
                vehicle_b.target_speed = vehicle_b.kinematics.v_levels[0]; // Slow speed
                vehicle_b.state = crate::components::VehicleState::Yielding;
            }
        }
    }

    /// Check for actual collisions in intersection using capsule detection
    fn check_intersection_collisions(
        &self,
        vehicles: &[PathVehicle],
        delta_time: f32,
        stats: &mut Stats,
    ) -> Vec<u64> {
        let mut collided_ids = Vec::new();
        
        for i in 0..vehicles.len() {
            for j in (i + 1)..vehicles.len() {
                let vehicle_a = &vehicles[i];
                let vehicle_b = &vehicles[j];
                
                if CapsuleCollision::check_intersection_collision(
                    vehicle_a,
                    vehicle_b,
                    delta_time,
                    self.intersection_bounds,
                ) {
                    stats.collisions += 1;
                    collided_ids.push(vehicle_a.id);
                    collided_ids.push(vehicle_b.id);
                }
            }
        }
        
        collided_ids
    }
}

impl Default for EnhancedCollisionSystem {
    fn default() -> Self {
        Self::new(Vector2::new(400.0, 300.0), 200.0) // Default intersection
    }
}

// Legacy system for backward compatibility (simplified without legion dependency)
pub fn collision_detection(vehicles: &[Vehicle], stats: &mut Stats) -> Vec<u64> {
    let mut vehicle_ids_to_remove = Vec::new();

    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let v1 = &vehicles[i];
            let v2 = &vehicles[j];

            // Enhanced collision course detection
            if enhanced_collision_course_check(v1, v2) {
                // Use capsule collision instead of simple AABB
                let collision = check_capsule_collision(v1, v2);

                if collision {
                    stats.collisions += 1;
                    vehicle_ids_to_remove.push(v1.id);
                    vehicle_ids_to_remove.push(v2.id);
                }
            }
        }
    }

    vehicle_ids_to_remove
}

/// Enhanced collision course detection
fn enhanced_collision_course_check(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
    let distance = (vehicle1.position - vehicle2.position).magnitude();
    if distance > 300.0 {
        return false;
    }

    // Check if they're in the intersection area
    let intersection_center = Vector2::new(400.0, 300.0); // Adjust to your intersection
    let intersection_radius = 150.0;
    
    let v1_to_center = (vehicle1.position - intersection_center).magnitude();
    let v2_to_center = (vehicle2.position - intersection_center).magnitude();
    
    let v1_in_intersection = v1_to_center < intersection_radius;
    let v2_in_intersection = v2_to_center < intersection_radius;
    
    if !v1_in_intersection && !v2_in_intersection {
        // Use original logic for vehicles outside intersection
        return VehiclePhysics::are_on_collision_course(vehicle1, vehicle2);
    }
    
    // For vehicles in intersection, use trajectory-based detection
    let v1_direction = vehicle1.direction.movement_vector();
    let v2_direction = vehicle2.direction.movement_vector();
    
    // Check if paths will intersect
    let future_distance = 100.0; // Look ahead distance
    let _v1_future = vehicle1.position + v1_direction * future_distance;
    let _v2_future = vehicle2.position + v2_direction * future_distance;
    
    // Calculate closest approach
    let rel_pos = vehicle2.position - vehicle1.position;
    let rel_vel = v2_direction * vehicle2.speed - v1_direction * vehicle1.speed;
    
    if rel_vel.magnitude() < 1.0 {
        return distance < 80.0; // Stationary case
    }
    
    let time_to_closest = -rel_pos.dot(&rel_vel) / rel_vel.magnitude_squared();
    if time_to_closest < 0.0 || time_to_closest > 5.0 {
        return false; // Closest approach is in the past or too far future
    }
    
    let closest_distance = (rel_pos + rel_vel * time_to_closest).magnitude();
    closest_distance < 60.0 // Collision threshold
}

/// Capsule-based collision detection for vehicles
fn check_capsule_collision(vehicle1: &Vehicle, vehicle2: &Vehicle) -> bool {
    let v1_center = vehicle1.position;
    let v2_center = vehicle2.position;
    
    let distance = (v1_center - v2_center).magnitude();
    let collision_distance = (vehicle1.width + vehicle2.width) / 2.0 + 10.0; // Safety margin
    
    distance < collision_distance
}
