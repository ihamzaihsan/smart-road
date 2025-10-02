use crate::{
    components::path_vehicle::PathVehicle,
    resources::stats::Stats,
    systems::{
        collision_system::EnhancedCollisionSystem,
        intersection_entry_system::IntersectionEntrySystem,
    },
};
use nalgebra::Vector2;

/// Advanced traffic controller with conflict-aware management
pub struct TrafficController {
    pub collision_system: EnhancedCollisionSystem,
    pub intersection_entry_system: IntersectionEntrySystem,
    pub vehicles: Vec<PathVehicle>,
    pub current_time: f32,
}

impl TrafficController {
    pub fn new(intersection_center: Vector2<f32>, intersection_size: f32) -> Self {
        Self {
            collision_system: EnhancedCollisionSystem::new(intersection_center, intersection_size),
            intersection_entry_system: IntersectionEntrySystem::new(
                intersection_center,
                intersection_size,
            ),
            vehicles: Vec::new(),
            current_time: 0.0,
        }
    }

    /// Main update loop for traffic management
    pub fn update(&mut self, delta_time: f32, stats: &mut Stats) -> Vec<u64> {
        self.current_time += delta_time;

        // Update intersection entry system first (manages queuing and priority access)
        self.intersection_entry_system.update(
            &mut self.vehicles,
            self.current_time,
            &mut self.collision_system.conflict_system,
        );

        // Update vehicle physics and positions separately to avoid borrow conflicts
        let vehicle_count = self.vehicles.len();
        for i in 0..vehicle_count {
            // Skip physics updates for waiting vehicles (they're managed by entry system)
            if self.vehicles[i].state == crate::components::VehicleState::Waiting {
                continue;
            }

            // Update speed control
            if let Some(target_eta) = self.vehicles[i].target_eta {
                self.vehicles[i].apply_speed_control(target_eta, self.current_time);
            }

            // Check emergency braking
            let needs_emergency_brake = self.check_emergency_brake_needed_by_index(i);
            if needs_emergency_brake {
                self.vehicles[i].target_speed = 0.0;
                self.vehicles[i].state = crate::components::VehicleState::Stopped;
                continue;
            }

            // Adaptive speed based on traffic density (only for moving vehicles)
            if self.vehicles[i].state == crate::components::VehicleState::Moving {
                let local_density = self.calculate_local_density_by_index(i);
                if local_density > 0.7 {
                    // High density - slow down
                    self.vehicles[i].target_speed =
                        self.vehicles[i].kinematics.v_levels[0].min(self.vehicles[i].target_speed);
                } else if local_density < 0.3
                    && self.vehicles[i].state != crate::components::VehicleState::Yielding
                {
                    // Low density - can speed up
                    self.vehicles[i].target_speed =
                        self.vehicles[i].kinematics.v_levels[2].min(300.0);
                }
            }

            // Reset yielding state if no longer needed
            if self.vehicles[i].state == crate::components::VehicleState::Yielding {
                if let Some(window) = self.vehicles[i].reserved_time_window {
                    if self.current_time > window.1 + 1.0 {
                        // Grace period after reservation expires
                        self.vehicles[i].state = crate::components::VehicleState::Moving;
                        self.vehicles[i].target_speed = self.vehicles[i].kinematics.v_levels[1];
                        // Medium speed
                    }
                }
            }

            // Update physics
            self.vehicles[i].update_speed(delta_time);
            self.vehicles[i].update_from_path(delta_time);
        }

        // Detect and resolve conflicts (for vehicles that have passed the entry system)
        let collided_ids = self.collision_system.detect_and_resolve_conflicts(
            &mut self.vehicles,
            self.current_time,
            delta_time,
            stats,
        );

        // Clean up completed or collided vehicles
        let removed_ids = collided_ids.clone();
        self.vehicles.retain(|v| {
            let should_retain = !v.is_path_complete() && !collided_ids.contains(&v.id);
            if !should_retain {
                // Remove vehicle from entry system queues when it's removed from simulation
                self.intersection_entry_system.remove_vehicle(v.id);
            }
            should_retain
        });

        removed_ids
    }

    /// Advanced vehicle controller with hysteresis and conflict awareness (legacy method)
    fn update_vehicle_controller(&mut self, vehicle: &mut PathVehicle, _delta_time: f32) {
        // Apply speed control based on reservations
        if let Some(target_eta) = vehicle.target_eta {
            vehicle.apply_speed_control(target_eta, self.current_time);
        }

        // Note: Emergency braking and density checks are now handled in the main update loop

        // Reset yielding state if no longer needed
        if vehicle.state == crate::components::VehicleState::Yielding {
            if let Some(window) = vehicle.reserved_time_window {
                if self.current_time > window.1 + 1.0 {
                    // Grace period after reservation expires
                    vehicle.state = crate::components::VehicleState::Moving;
                    vehicle.target_speed = vehicle.kinematics.v_levels[1]; // Medium speed
                }
            }
        }
    }

    /// Check if emergency braking is needed for vehicle at index
    fn check_emergency_brake_needed_by_index(&self, vehicle_index: usize) -> bool {
        let vehicle = &self.vehicles[vehicle_index];
        let emergency_distance = 50.0;
        let emergency_ttc = 1.5; // seconds

        for (i, other) in self.vehicles.iter().enumerate() {
            if i == vehicle_index {
                continue;
            }

            let distance = (vehicle.position - other.position).magnitude();
            if distance < emergency_distance {
                if let Some(ttc) =
                    crate::systems::capsule_collision::CapsuleCollision::calculate_ttc(
                        vehicle, other,
                    )
                {
                    if ttc < emergency_ttc {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Calculate local traffic density around vehicle at index
    fn calculate_local_density_by_index(&self, vehicle_index: usize) -> f32 {
        let vehicle = &self.vehicles[vehicle_index];
        let radius = 100.0;
        let max_vehicles = 8.0; // Expected max vehicles in radius

        let nearby_count = self
            .vehicles
            .iter()
            .enumerate()
            .filter(|(i, other)| {
                *i != vehicle_index
                    && matches!(
                        other.state,
                        crate::components::VehicleState::Moving
                            | crate::components::VehicleState::Yielding
                    )
                    && (other.position - vehicle.position).magnitude() < radius
            })
            .count() as f32;

        (nearby_count / max_vehicles).min(1.0)
    }

    /// Add a new vehicle to the traffic system
    pub fn add_vehicle(&mut self, vehicle: PathVehicle) {
        self.vehicles.push(vehicle);
    }

    /// Get intersection entry queue status for a specific direction
    pub fn get_queue_status(&self, direction: crate::components::Direction) -> (usize, bool) {
        if let Some(queue) = self.intersection_entry_system.entry_queues.get(&direction) {
            (queue.vehicles.len(), queue.current_serving.is_some())
        } else {
            (0, false)
        }
    }

    /// Force release a specific vehicle from queue (for debugging/emergency)
    pub fn force_release_vehicle(&mut self, vehicle_id: u64) {
        self.intersection_entry_system.remove_vehicle(vehicle_id);
        if let Some(vehicle) = self.vehicles.iter_mut().find(|v| v.id == vehicle_id) {
            vehicle.state = crate::components::VehicleState::Moving;
            vehicle.target_speed = vehicle.kinematics.v_levels[1]; // Medium speed
        }
    }

    /// Get telemetry data for debugging
    pub fn get_telemetry(&self) -> TrafficTelemetry {
        let mut telemetry = TrafficTelemetry::default();

        for vehicle in &self.vehicles {
            telemetry.total_vehicles += 1;

            match vehicle.state {
                crate::components::VehicleState::Moving => telemetry.moving_vehicles += 1,
                crate::components::VehicleState::Yielding => telemetry.yielding_vehicles += 1,
                crate::components::VehicleState::Stopped => telemetry.stopped_vehicles += 1,
                crate::components::VehicleState::Waiting => telemetry.waiting_vehicles += 1,
                crate::components::VehicleState::AlwaysMoving => telemetry.moving_vehicles += 1, // Count as moving
            }

            if vehicle.next_conflict_cell.is_some() {
                telemetry.vehicles_with_reservations += 1;
            }

            // Calculate average speed
            telemetry.average_speed += vehicle.speed;
        }

        if telemetry.total_vehicles > 0 {
            telemetry.average_speed /= telemetry.total_vehicles as f32;
        }

        telemetry.reservation_count = self
            .collision_system
            .conflict_system
            .cell_calendars
            .values()
            .map(|calendar| calendar.len())
            .sum();

        // Add queue telemetry
        let queue_telemetry = self.intersection_entry_system.get_queue_telemetry();
        telemetry.intersection_queue_data = Some(queue_telemetry);

        telemetry
    }

    /// Clear all vehicles and reset system
    pub fn reset(&mut self) {
        self.vehicles.clear();
        self.collision_system.conflict_system.cell_calendars.clear();
        self.intersection_entry_system.reset();
        self.current_time = 0.0;
    }
}

/// Telemetry data for monitoring traffic system performance
#[derive(Debug, Default)]
pub struct TrafficTelemetry {
    pub total_vehicles: usize,
    pub moving_vehicles: usize,
    pub yielding_vehicles: usize,
    pub stopped_vehicles: usize,
    pub waiting_vehicles: usize,
    pub vehicles_with_reservations: usize,
    pub reservation_count: usize,
    pub average_speed: f32,
    pub intersection_queue_data:
        Option<crate::systems::intersection_entry_system::IntersectionQueueTelemetry>,
}

impl TrafficTelemetry {
    /// Print telemetry to console for debugging
    pub fn print_debug(&self) {
        println!("=== Traffic Telemetry ===");
        println!("Total vehicles: {}", self.total_vehicles);
        println!(
            "Moving: {}, Yielding: {}, Stopped: {}, Waiting: {}",
            self.moving_vehicles,
            self.yielding_vehicles,
            self.stopped_vehicles,
            self.waiting_vehicles
        );
        println!(
            "Vehicles with reservations: {}",
            self.vehicles_with_reservations
        );
        println!("Active reservations: {}", self.reservation_count);
        println!("Average speed: {:.1} m/s", self.average_speed);

        // Print intersection queue information
        if let Some(ref queue_data) = self.intersection_queue_data {
            println!();
            queue_data.print_debug();
        }

        println!("========================");
    }
}
