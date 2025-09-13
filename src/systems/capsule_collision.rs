use nalgebra::Vector2;

/// Capsule collision detection for moving vehicles
pub struct CapsuleCollision;

impl CapsuleCollision {
    /// Check collision between two moving capsules over a time frame
    pub fn moving_capsules_collide(
        a0: Vector2<f32>, a1: Vector2<f32>,  // Vehicle A start and end positions
        b0: Vector2<f32>, b1: Vector2<f32>,  // Vehicle B start and end positions
        ra: f32, rb: f32,                    // Radii (half-width + margin)
        max_step: f32,                       // Max step size for sampling
    ) -> bool {
        let la = (a1 - a0).magnitude();
        let lb = (b1 - b0).magnitude();
        let max_distance = la.max(lb);
        
        if max_distance < 0.1 {
            // Both vehicles barely moving, just check static distance
            return (a0 - b0).magnitude() < ra + rb;
        }
        
        let n = ((max_distance / max_step).ceil() as i32).clamp(2, 8);
        
        for k in 0..n {
            let t = (k as f32) / (n as f32 - 1.0);
            let a_pos = a0.lerp(&a1, t);
            let b_pos = b0.lerp(&b1, t);
            
            let distance = (a_pos - b_pos).magnitude();
            if distance < ra + rb {
                return true;
            }
        }
        
        false
    }

    /// Calculate minimum distance between two line segments
    pub fn segment_distance(a0: Vector2<f32>, a1: Vector2<f32>, b0: Vector2<f32>, b1: Vector2<f32>) -> f32 {
        let u = a1 - a0;
        let v = b1 - b0;
        let w = a0 - b0;
        
        let a = u.dot(&u);  // |u|^2
        let b = u.dot(&v);
        let c = v.dot(&v);  // |v|^2
        let d = u.dot(&w);
        let e = v.dot(&w);
        
        let denom = a * c - b * b;
        let mut sc: f32;
        let mut tc: f32;
        
        if denom < 1e-6 {
            // Lines are nearly parallel
            sc = 0.0;
            tc = if b > c { d / b } else { e / c };
        } else {
            sc = (b * e - c * d) / denom;
            tc = (a * e - b * d) / denom;
        }
        
        // Clamp to segment bounds
        sc = sc.clamp(0.0, 1.0);
        tc = tc.clamp(0.0, 1.0);
        
        let closest_a = a0 + sc * u;
        let closest_b = b0 + tc * v;
        
        (closest_a - closest_b).magnitude()
    }

    /// Enhanced collision check for intersection area with vehicle orientation
    pub fn check_intersection_collision(
        vehicle_a: &crate::components::path_vehicle::PathVehicle,
        vehicle_b: &crate::components::path_vehicle::PathVehicle,
        delta_time: f32,
        intersection_bounds: (Vector2<f32>, Vector2<f32>), // (min, max)
    ) -> bool {
        // Check if both vehicles are in or near intersection
        let margin = 100.0; // Margin around intersection
        let bounds_min = intersection_bounds.0 - Vector2::new(margin, margin);
        let bounds_max = intersection_bounds.1 + Vector2::new(margin, margin);
        
        let a_in_area = Self::point_in_bounds(vehicle_a.position, bounds_min, bounds_max);
        let b_in_area = Self::point_in_bounds(vehicle_b.position, bounds_min, bounds_max);
        
        if !a_in_area && !b_in_area {
            return false;
        }
        
        // Calculate future positions
        let a_velocity = Vector2::new(
            vehicle_a.rotation.cos() * vehicle_a.speed,
            vehicle_a.rotation.sin() * vehicle_a.speed,
        );
        let b_velocity = Vector2::new(
            vehicle_b.rotation.cos() * vehicle_b.speed,
            vehicle_b.rotation.sin() * vehicle_b.speed,
        );
        
        let a_future = vehicle_a.position + a_velocity * delta_time;
        let b_future = vehicle_b.position + b_velocity * delta_time;
        
        // Vehicle dimensions (add safety margin)
        let safety_margin = 15.0;
        let ra = (vehicle_a.width / 2.0) + safety_margin;
        let rb = (vehicle_b.width / 2.0) + safety_margin;
        
        // Use capsule collision with smaller substeps for accuracy
        let max_step = 5.0; // 5 meters per substep
        
        Self::moving_capsules_collide(
            vehicle_a.position,
            a_future,
            vehicle_b.position,
            b_future,
            ra,
            rb,
            max_step,
        )
    }

    fn point_in_bounds(point: Vector2<f32>, min: Vector2<f32>, max: Vector2<f32>) -> bool {
        point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y
    }

    /// Calculate Time-to-Collision (TTC) for two vehicles
    pub fn calculate_ttc(
        vehicle_a: &crate::components::path_vehicle::PathVehicle,
        vehicle_b: &crate::components::path_vehicle::PathVehicle,
    ) -> Option<f32> {
        let rel_pos = vehicle_b.position - vehicle_a.position;
        let distance = rel_pos.magnitude();
        
        if distance < 0.1 {
            return Some(0.0); // Already colliding
        }
        
        let a_velocity = Vector2::new(
            vehicle_a.rotation.cos() * vehicle_a.speed,
            vehicle_a.rotation.sin() * vehicle_a.speed,
        );
        let b_velocity = Vector2::new(
            vehicle_b.rotation.cos() * vehicle_b.speed,
            vehicle_b.rotation.sin() * vehicle_b.speed,
        );
        
        let rel_velocity = a_velocity - b_velocity;
        let closing_speed = -rel_pos.dot(&rel_velocity) / distance;
        
        if closing_speed <= 0.0 {
            return None; // Vehicles are separating
        }
        
        let collision_distance = (vehicle_a.width + vehicle_b.width) / 2.0 + 10.0; // Safety margin
        let ttc = (distance - collision_distance) / closing_speed;
        
        if ttc > 0.0 {
            Some(ttc)
        } else {
            Some(0.0) // Already within collision distance
        }
    }

    /// Check if two vehicles are on a crossing path (not following)
    pub fn are_crossing(&self, 
        vehicle_a: &crate::components::path_vehicle::PathVehicle,
        vehicle_b: &crate::components::path_vehicle::PathVehicle,
    ) -> bool {
        // Check if they have different approaches but same conflict cell
        if vehicle_a.approach != vehicle_b.approach {
            // Different approaches - likely crossing
            match (&vehicle_a.next_conflict_cell, &vehicle_b.next_conflict_cell) {
                (Some(cell_a), Some(cell_b)) => cell_a == cell_b,
                _ => false,
            }
        } else {
            // Same approach - check if different routes that might conflict
            vehicle_a.route != vehicle_b.route
        }
    }
}

/// Time-to-Collision analysis for conflict prediction
pub struct TTCAnalysis;

impl TTCAnalysis {
    /// Calculate TTC to a specific conflict cell
    pub fn ttc_to_conflict_cell(
        vehicle: &crate::components::path_vehicle::PathVehicle,
        cell_span: &crate::systems::conflict_system::ConflictSpan,
    ) -> f32 {
        let eta_to_cell = vehicle.eta_to_s(cell_span.s_in);
        eta_to_cell
    }

    /// Check if two vehicles have conflicting ETAs to the same cell
    pub fn check_eta_conflict(
        vehicle_a: &crate::components::path_vehicle::PathVehicle,
        vehicle_b: &crate::components::path_vehicle::PathVehicle,
        cell_span_a: &crate::systems::conflict_system::ConflictSpan,
        cell_span_b: &crate::systems::conflict_system::ConflictSpan,
        separation_time: f32,
    ) -> bool {
        if cell_span_a.cell != cell_span_b.cell {
            return false;
        }
        
        let eta_a = Self::ttc_to_conflict_cell(vehicle_a, cell_span_a);
        let eta_b = Self::ttc_to_conflict_cell(vehicle_b, cell_span_b);
        
        // Calculate occupation durations
        let duration_a = (cell_span_a.s_out - cell_span_a.s_in) / vehicle_a.speed.max(1.0);
        let duration_b = (cell_span_b.s_out - cell_span_b.s_in) / vehicle_b.speed.max(1.0);
        
        // Check if time windows overlap
        let a_start = eta_a;
        let a_end = eta_a + duration_a;
        let b_start = eta_b;
        let b_end = eta_b + duration_b;
        
        // Windows conflict if they overlap with insufficient separation
        !(a_end + separation_time <= b_start || b_end + separation_time <= a_start)
    }

    /// Determine which vehicle should yield based on priority rules
    pub fn determine_yielding_vehicle(
        vehicle_a: &crate::components::path_vehicle::PathVehicle,
        vehicle_b: &crate::components::path_vehicle::PathVehicle,
        eta_a: f32,
        eta_b: f32,
    ) -> u64 {
        const ETA_TOLERANCE: f32 = 0.1; // 100ms tolerance
        
        if (eta_a - eta_b).abs() > ETA_TOLERANCE {
            // Clear ETA difference - later arrival yields
            if eta_a > eta_b {
                vehicle_a.id
            } else {
                vehicle_b.id
            }
        } else {
            // ETAs are close - use priority rules
            let priority_a = crate::systems::conflict_system::ConflictSystem::calculate_priority(
                vehicle_a.approach, vehicle_a.route, vehicle_a.id
            );
            let priority_b = crate::systems::conflict_system::ConflictSystem::calculate_priority(
                vehicle_b.approach, vehicle_b.route, vehicle_b.id
            );
            
            if priority_a >= priority_b {
                vehicle_b.id // Lower priority yields
            } else {
                vehicle_a.id
            }
        }
    }
}
