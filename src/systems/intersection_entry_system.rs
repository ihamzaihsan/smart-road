use crate::{
    components::{path_vehicle::PathVehicle, Direction, Route, VehicleState},
    systems::conflict_system::ConflictSystem,
};
use nalgebra::Vector2;
use std::collections::{HashMap, VecDeque};

/// Represents a vehicle waiting to enter the intersection
#[derive(Debug, Clone)]
pub struct QueuedVehicle {
    pub vehicle_id: u64,
    pub approach: Direction,
    pub route: Route,
    pub queue_entry_time: f32,
    pub priority_level: u32,
    pub waiting_position: Vector2<f32>,
}

/// Priority levels for intersection entry
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum EntryPriority {
    Emergency = 4, // Emergency vehicles (future use)
    Right = 3,     // Right turns - highest normal priority
    Straight = 2,  // Straight through
    Left = 1,      // Left turns - lowest priority
}

impl From<Route> for EntryPriority {
    fn from(route: Route) -> Self {
        match route {
            Route::Right => EntryPriority::Right,
            Route::Straight => EntryPriority::Straight,
            Route::Left => EntryPriority::Left,
        }
    }
}

/// Queue for each approach direction and lane
#[derive(Debug, Default)]
pub struct LaneQueue {
    pub vehicles: VecDeque<QueuedVehicle>,
    pub current_serving: Option<u64>, // Vehicle currently being processed for entry
}

/// System managing intersection entry queues and priorities
pub struct IntersectionEntrySystem {
    /// Queue for each approach direction - vehicles wait here before entering intersection
    pub entry_queues: HashMap<Direction, LaneQueue>,
    /// Position where vehicles should wait at each approach (lane separator positions)
    pub waiting_positions: HashMap<Direction, Vector2<f32>>,
    /// Intersection bounds for determining when a vehicle needs to queue
    pub intersection_bounds: (Vector2<f32>, Vector2<f32>), // (min, max)
    /// Current time
    pub current_time: f32,
    /// Maximum time a vehicle can wait before being force-processed
    pub max_wait_time: f32,
    /// Distance before intersection where vehicles start queuing
    pub queue_start_distance: f32,
    /// Minimum spacing maintained between vehicles in the same queue
    pub queue_spacing_min: f32,
    /// Extra buffer added on top of individual vehicle length for queue spacing
    pub queue_spacing_buffer: f32,
    /// Distance threshold used to decide if the serving vehicle still blocks the waiting line
    pub serving_clearance_distance: f32,
}

impl IntersectionEntrySystem {
    pub fn new(intersection_center: Vector2<f32>, intersection_size: f32) -> Self {
        let half_size = intersection_size / 2.0;
        let bounds = (
            intersection_center - Vector2::new(half_size, half_size),
            intersection_center + Vector2::new(half_size, half_size),
        );

        // Define waiting positions at lane separators for each approach
        let queue_distance = intersection_size * 0.8; // Distance from center to waiting line
        let mut waiting_positions = HashMap::new();

        waiting_positions.insert(
            Direction::North,
            intersection_center + Vector2::new(0.0, -queue_distance), // Approaching from south
        );
        waiting_positions.insert(
            Direction::South,
            intersection_center + Vector2::new(0.0, queue_distance), // Approaching from north
        );
        waiting_positions.insert(
            Direction::East,
            intersection_center + Vector2::new(-queue_distance, 0.0), // Approaching from west
        );
        waiting_positions.insert(
            Direction::West,
            intersection_center + Vector2::new(queue_distance, 0.0), // Approaching from east
        );

        // Initialize empty queues for each direction
        let mut entry_queues = HashMap::new();
        for direction in [
            Direction::North,
            Direction::South,
            Direction::East,
            Direction::West,
        ] {
            entry_queues.insert(direction, LaneQueue::default());
        }

        Self {
            entry_queues,
            waiting_positions,
            intersection_bounds: bounds,
            current_time: 0.0,
            max_wait_time: 10.0, // 10 seconds max wait time
            queue_start_distance: queue_distance,
            queue_spacing_min: 140.0,        // Increased from 90.0 for better spacing
            queue_spacing_buffer: 40.0,      // Increased from 20.0 for safety margin
            serving_clearance_distance: 120.0, // Increased from 70.0 to give more space
        }
    }

    /// Update the entry system for each frame
    pub fn update(
        &mut self,
        vehicles: &mut [PathVehicle],
        current_time: f32,
        conflict_system: &mut ConflictSystem,
    ) {
        self.current_time = current_time;

        // First, identify vehicles that should be queued
        for vehicle in vehicles.iter_mut() {
            if self.should_enter_queue(vehicle) && vehicle.state != VehicleState::Waiting {
                self.add_vehicle_to_queue(vehicle);
            }
        }

        // Process each direction's queue
        for direction in [
            Direction::North,
            Direction::South,
            Direction::East,
            Direction::West,
        ] {
            self.process_direction_queue(direction, vehicles, conflict_system);
        }

        // Update vehicle states and positions for queued vehicles
        self.update_queued_vehicle_positions(vehicles);
    }

    /// Determine if a vehicle should enter the entry queue
    fn should_enter_queue(&self, vehicle: &PathVehicle) -> bool {
        // Don't queue vehicles that are already waiting or have completed more than 70% of their path
        if vehicle.state == VehicleState::Waiting || vehicle.path_progress() > 0.7 {
            return false;
        }

        if let Some(queue) = self.entry_queues.get(&vehicle.approach) {
            if queue.current_serving == Some(vehicle.id) {
                // Vehicle has already been granted access, don't requeue while it's still being served
                return false;
            }

            if queue
                .vehicles
                .iter()
                .any(|queued| queued.vehicle_id == vehicle.id)
            {
                // Vehicle is already enqueued
                return false;
            }
        }

        // Check if vehicle is approaching the intersection
        let approach_position = self
            .waiting_positions
            .get(&vehicle.approach)
            .copied()
            .unwrap_or(Vector2::zeros());

        let distance_to_queue_line = (vehicle.position - approach_position).magnitude();

        // Vehicle should queue if it's close to the intersection but not yet in it
        distance_to_queue_line < self.queue_start_distance / 2.0
            && !self.is_vehicle_in_intersection(vehicle)
    }

    /// Check if vehicle is currently inside the intersection bounds
    fn is_vehicle_in_intersection(&self, vehicle: &PathVehicle) -> bool {
        let (min_bounds, max_bounds) = self.intersection_bounds;
        vehicle.position.x >= min_bounds.x
            && vehicle.position.x <= max_bounds.x
            && vehicle.position.y >= min_bounds.y
            && vehicle.position.y <= max_bounds.y
    }

    /// Add a vehicle to the appropriate entry queue
    fn add_vehicle_to_queue(&mut self, vehicle: &mut PathVehicle) {
        let priority = EntryPriority::from(vehicle.route) as u32;
        let waiting_position = self
            .waiting_positions
            .get(&vehicle.approach)
            .copied()
            .unwrap_or(vehicle.position);

        let queued_vehicle = QueuedVehicle {
            vehicle_id: vehicle.id,
            approach: vehicle.approach,
            route: vehicle.route,
            queue_entry_time: self.current_time,
            priority_level: priority,
            waiting_position,
        };

        // Get approach vector before borrowing queue mutably
        let approach_vector = self.get_approach_vector(vehicle.approach);
        
        // Calculate the position for this vehicle BEFORE adding to queue
        // This ensures the vehicle immediately goes to the correct position
        let (queue_length, has_serving) = if let Some(queue) = self.entry_queues.get(&vehicle.approach) {
            (queue.vehicles.len(), queue.current_serving.is_some())
        } else {
            return; // No queue for this direction
        };
        
        // Calculate offset based on existing queue
        let mut offset = if has_serving {
            // If there's a serving vehicle blocking the line, add extra offset
            self.queue_spacing_min
        } else {
            0.0
        };
        
        // Add offset for all existing vehicles in queue
        offset += (queue_length as f32) * self.queue_spacing_min;
        
        // Set vehicle position IMMEDIATELY to prevent collision while approaching
        // Use same formula as update_queued_vehicle_positions for consistency
        vehicle.position = waiting_position - approach_vector * offset;
        vehicle.speed = 0.0;
        vehicle.target_speed = 0.0;
        vehicle.state = VehicleState::Waiting;
        
        // Now add to queue
        if let Some(queue) = self.entry_queues.get_mut(&vehicle.approach) {
            queue.vehicles.push_back(queued_vehicle);
        }
    }

    /// Process the queue for a specific direction
    fn process_direction_queue(
        &mut self,
        direction: Direction,
        vehicles: &mut [PathVehicle],
        conflict_system: &mut ConflictSystem,
    ) {
        // First, check what needs to be done
        let (should_try_serve, _current_serving, needs_clear_serving) = {
            if let Some(queue) = self.entry_queues.get(&direction) {
                let should_try = queue.current_serving.is_none() && !queue.vehicles.is_empty();
                let serving = queue.current_serving;
                let needs_clear = if let Some(serving_id) = serving {
                    if let Some(vehicle) = vehicles.iter().find(|v| v.id == serving_id) {
                        self.has_vehicle_cleared_intersection(vehicle)
                    } else {
                        true // Vehicle no longer exists
                    }
                } else {
                    false
                };
                (should_try, serving, needs_clear)
            } else {
                return;
            }
        };

        // Clear serving if needed
        if needs_clear_serving {
            if let Some(queue) = self.entry_queues.get_mut(&direction) {
                if let Some(serving_id) = queue.current_serving {
                    println!("Vehicle {} has cleared the intersection", serving_id);
                }
                queue.current_serving = None;
            }
        }

        // Try to serve next vehicle if needed
        if should_try_serve {
            let next_vehicle_info = {
                if let Some(queue) = self.entry_queues.get_mut(&direction) {
                    // Sort queue by priority and then by arrival time (FIFO within priority)
                    queue.vehicles.make_contiguous().sort_by(|a, b| {
                        // Higher priority first, then earlier arrival time
                        b.priority_level.cmp(&a.priority_level).then(
                            a.queue_entry_time
                                .partial_cmp(&b.queue_entry_time)
                                .unwrap_or(std::cmp::Ordering::Equal),
                        )
                    });

                    queue.vehicles.front().cloned()
                } else {
                    None
                }
            };

            if let Some(next_vehicle) = next_vehicle_info {
                if self.can_grant_intersection_access(&next_vehicle, vehicles, conflict_system) {
                    // Grant access
                    if let Some(queue) = self.entry_queues.get_mut(&direction) {
                        let vehicle_to_serve = queue.vehicles.pop_front().unwrap();
                        queue.current_serving = Some(vehicle_to_serve.vehicle_id);

                        // Find and update the vehicle
                        if let Some(vehicle) = vehicles
                            .iter_mut()
                            .find(|v| v.id == vehicle_to_serve.vehicle_id)
                        {
                            vehicle.state = VehicleState::Moving;
                            vehicle.target_speed = vehicle.kinematics.v_levels[1]; // Medium speed
                            println!(
                                "Granting intersection access to vehicle {} from {:?} going {:?}",
                                vehicle.id, vehicle.approach, vehicle.route
                            );
                        }
                    }
                }
            }
        }

        // Handle vehicles that have waited too long (prevent deadlock)
        let current_time = self.current_time;
        let max_wait = self.max_wait_time;

        if let Some(queue) = self.entry_queues.get_mut(&direction) {
            queue.vehicles.retain(|qv| {
                if current_time - qv.queue_entry_time > max_wait {
                    println!(
                        "Force-releasing vehicle {} due to long wait time",
                        qv.vehicle_id
                    );
                    // Find and release the vehicle
                    if let Some(vehicle) = vehicles.iter_mut().find(|v| v.id == qv.vehicle_id) {
                        vehicle.state = VehicleState::Moving;
                        vehicle.target_speed = vehicle.kinematics.v_levels[0]; // Slow speed
                    }
                    false // Remove from queue
                } else {
                    true // Keep in queue
                }
            });
        }
    }

    /// Check if intersection access can be granted to a vehicle
    fn can_grant_intersection_access(
        &self,
        queued_vehicle: &QueuedVehicle,
        vehicles: &[PathVehicle],
        conflict_system: &ConflictSystem,
    ) -> bool {
        // First, check if ANY vehicle is currently being served from ANY direction
        // This implements a simple "one at a time" policy for maximum safety
        for (_direction, queue) in &self.entry_queues {
            if queue.current_serving.is_some() {
                return false; // Someone else is already being served
            }
        }

        // Check if there are any vehicles currently in the intersection
        for vehicle in vehicles {
            if vehicle.id == queued_vehicle.vehicle_id {
                continue;
            }

            // If another vehicle is in the intersection, deny access
            if self.is_vehicle_in_intersection(vehicle) {
                return false;
            }
        }

        // Additional conflict check for vehicles that might enter intersection soon
        for vehicle in vehicles {
            if vehicle.id == queued_vehicle.vehicle_id {
                continue;
            }

            // Check if vehicle is close to intersection and would conflict
            if vehicle.state == VehicleState::Moving {
                let distance_to_center = (vehicle.position
                    - Vector2::new(
                        (self.intersection_bounds.0.x + self.intersection_bounds.1.x) / 2.0,
                        (self.intersection_bounds.0.y + self.intersection_bounds.1.y) / 2.0,
                    ))
                .magnitude();

                if distance_to_center < self.queue_start_distance * 1.5 {
                    // Create a mock queued vehicle for conflict checking
                    let _mock_queued = QueuedVehicle {
                        vehicle_id: vehicle.id,
                        approach: vehicle.approach,
                        route: vehicle.route,
                        queue_entry_time: 0.0,
                        priority_level: EntryPriority::from(vehicle.route) as u32,
                        waiting_position: Vector2::zeros(),
                    };

                    if self.would_paths_conflict(queued_vehicle, vehicle, conflict_system) {
                        return false;
                    }
                }
            }
        }

        true
    }

    /// Check if two vehicles' paths would conflict in the intersection
    fn would_paths_conflict(
        &self,
        queued_vehicle: &QueuedVehicle,
        other_vehicle: &PathVehicle,
        conflict_system: &ConflictSystem,
    ) -> bool {
        // Get conflict spans for both vehicles
        let queued_spans =
            conflict_system.get_conflict_spans(queued_vehicle.approach, queued_vehicle.route);
        let other_spans =
            conflict_system.get_conflict_spans(other_vehicle.approach, other_vehicle.route);

        if let (Some(spans1), Some(spans2)) = (queued_spans, other_spans) {
            // Check if any conflict cells overlap
            for span1 in spans1 {
                for span2 in spans2 {
                    if span1.cell == span2.cell {
                        return true; // Paths conflict at this cell
                    }
                }
            }
        }

        false
    }

    /// Check if a vehicle has cleared the intersection
    fn has_vehicle_cleared_intersection(&self, vehicle: &PathVehicle) -> bool {
        let progress = vehicle.path_progress();

        if progress >= 0.92 {
            return true;
        }

        if progress <= 0.25 {
            // Vehicle hasn't yet meaningfully entered the intersection
            return false;
        }

        // Vehicle has cleared if it has moved sufficiently far beyond the entry line
        let (min_bounds, max_bounds) = self.intersection_bounds;
        let is_outside = vehicle.position.x < min_bounds.x
            || vehicle.position.x > max_bounds.x
            || vehicle.position.y < min_bounds.y
            || vehicle.position.y > max_bounds.y;

        is_outside || progress > 0.75
    }

    /// Update positions of vehicles waiting in queue
    fn update_queued_vehicle_positions(&self, vehicles: &mut [PathVehicle]) {
        for (direction, queue) in &self.entry_queues {
            let Some(&waiting_pos) = self.waiting_positions.get(direction) else {
                continue;
            };

            let approach_vector = self.get_approach_vector(*direction);

            // Determine if the currently served vehicle is still blocking the waiting line
            let mut offset = if let Some(serving_id) = queue.current_serving {
                vehicles
                    .iter()
                    .find(|v| v.id == serving_id)
                    .and_then(|serving_vehicle| {
                        let distance = (serving_vehicle.position - waiting_pos).magnitude();
                        if distance < self.serving_clearance_distance {
                            Some(self.queue_spacing_min)
                        } else {
                            None
                        }
                    })
                    .unwrap_or(0.0)
            } else {
                0.0
            };

            // Process ALL queued vehicles, regardless of their current state
            // Force them into Waiting state with correct spacing
            for queued_vehicle in &queue.vehicles {
                if let Some(vehicle) = vehicles
                    .iter_mut()
                    .find(|v| v.id == queued_vehicle.vehicle_id)
                {
                    // Calculate spacing for this vehicle
                    let spacing = (vehicle.length + self.queue_spacing_buffer)
                        .max(self.queue_spacing_min);

                    // Force vehicle into proper queue state and position
                    vehicle.position = waiting_pos - approach_vector * offset;
                    vehicle.speed = 0.0;
                    vehicle.target_speed = 0.0;
                    // Force state to Waiting - prevents collision system from interfering
                    vehicle.state = VehicleState::Waiting;

                    // Accumulate offset for next vehicle in queue
                    offset += spacing;
                }
            }
        }
    }

    /// Get the approach vector for a direction (unit vector pointing toward intersection)
    fn get_approach_vector(&self, direction: Direction) -> Vector2<f32> {
        match direction {
            Direction::North => Vector2::new(0.0, 1.0), // Moving north (up)
            Direction::South => Vector2::new(0.0, -1.0), // Moving south (down)
            Direction::East => Vector2::new(1.0, 0.0),  // Moving east (right)
            Direction::West => Vector2::new(-1.0, 0.0), // Moving west (left)
        }
    }

    /// Get current queue status for debugging
    pub fn get_queue_telemetry(&self) -> IntersectionQueueTelemetry {
        let mut telemetry = IntersectionQueueTelemetry::default();

        for (direction, queue) in &self.entry_queues {
            let queue_size = queue.vehicles.len();
            let is_serving = queue.current_serving.is_some();

            match direction {
                Direction::North => {
                    telemetry.north_queue_size = queue_size;
                    telemetry.north_serving = is_serving;
                }
                Direction::South => {
                    telemetry.south_queue_size = queue_size;
                    telemetry.south_serving = is_serving;
                }
                Direction::East => {
                    telemetry.east_queue_size = queue_size;
                    telemetry.east_serving = is_serving;
                }
                Direction::West => {
                    telemetry.west_queue_size = queue_size;
                    telemetry.west_serving = is_serving;
                }
            }

            telemetry.total_queued += queue_size;
        }

        telemetry
    }

    /// Remove a vehicle from all queues (called when vehicle is removed from simulation)
    pub fn remove_vehicle(&mut self, vehicle_id: u64) {
        for queue in self.entry_queues.values_mut() {
            queue.vehicles.retain(|qv| qv.vehicle_id != vehicle_id);
            if queue.current_serving == Some(vehicle_id) {
                queue.current_serving = None;
            }
        }
    }

    /// Reset all queues
    pub fn reset(&mut self) {
        for queue in self.entry_queues.values_mut() {
            queue.vehicles.clear();
            queue.current_serving = None;
        }
        self.current_time = 0.0;
    }
}

/// Telemetry data for intersection queue monitoring
#[derive(Debug, Default)]
pub struct IntersectionQueueTelemetry {
    pub total_queued: usize,
    pub north_queue_size: usize,
    pub south_queue_size: usize,
    pub east_queue_size: usize,
    pub west_queue_size: usize,
    pub north_serving: bool,
    pub south_serving: bool,
    pub east_serving: bool,
    pub west_serving: bool,
}

impl IntersectionQueueTelemetry {
    pub fn print_debug(&self) {
        println!("=== Intersection Queue Status ===");
        println!("Total vehicles queued: {}", self.total_queued);
        println!(
            "North: {} queued, serving: {}",
            self.north_queue_size, self.north_serving
        );
        println!(
            "South: {} queued, serving: {}",
            self.south_queue_size, self.south_serving
        );
        println!(
            "East: {} queued, serving: {}",
            self.east_queue_size, self.east_serving
        );
        println!(
            "West: {} queued, serving: {}",
            self.west_queue_size, self.west_serving
        );
        println!("================================");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::systems::conflict_system::{ConflictSystem, Path};

    fn make_test_vehicle(id: u64, approach: Direction) -> PathVehicle {
        let start = match approach {
            Direction::North => Vector2::new(400.0, 100.0),
            Direction::South => Vector2::new(400.0, 500.0),
            Direction::East => Vector2::new(100.0, 300.0),
            Direction::West => Vector2::new(700.0, 300.0),
        };

        let end = match approach {
            Direction::North => Vector2::new(400.0, 500.0),
            Direction::South => Vector2::new(400.0, 100.0),
            Direction::East => Vector2::new(700.0, 300.0),
            Direction::West => Vector2::new(100.0, 300.0),
        };

        let path = Path::new(vec![start, end]);

        let mut vehicle = PathVehicle::new(id, start, approach, Route::Straight, path, 0);
        vehicle.state = VehicleState::Waiting;
        vehicle.speed = 0.0;
        vehicle.target_speed = 0.0;
        vehicle
    }

    #[test]
    fn queued_vehicles_are_evenly_spaced() {
        let mut system = IntersectionEntrySystem::new(Vector2::new(400.0, 300.0), 200.0);
        let mut vehicles = vec![
            make_test_vehicle(1, Direction::North),
            make_test_vehicle(2, Direction::North),
            make_test_vehicle(3, Direction::North),
        ];

        for vehicle in &mut vehicles {
            system.add_vehicle_to_queue(vehicle);
        }

        system.update_queued_vehicle_positions(&mut vehicles);

        let waiting_pos = system.waiting_positions[&Direction::North];
        let spacing = system.queue_spacing_min;

        assert!((vehicles[0].position.y - waiting_pos.y).abs() < 1.0);
        assert!((vehicles[1].position.y - (waiting_pos.y - spacing)).abs() < 1.0);
        assert!((vehicles[2].position.y - (waiting_pos.y - 2.0 * spacing)).abs() < 1.5);
    }

    #[test]
    fn queue_respects_current_serving_clearance() {
        let mut system = IntersectionEntrySystem::new(Vector2::new(400.0, 300.0), 200.0);
        let mut vehicles = vec![
            make_test_vehicle(10, Direction::North),
            make_test_vehicle(11, Direction::North),
            make_test_vehicle(12, Direction::North),
        ];

        system.add_vehicle_to_queue(&mut vehicles[0]);
        system.add_vehicle_to_queue(&mut vehicles[1]);

        // Simulate serving the first vehicle while it is still near the waiting line
        if let Some(queue) = system.entry_queues.get_mut(&Direction::North) {
            let served = queue.vehicles.pop_front().unwrap();
            queue.current_serving = Some(served.vehicle_id);
        }
        vehicles[0].state = VehicleState::Moving;
        if let Some(&waiting_pos) = system.waiting_positions.get(&Direction::North) {
            vehicles[0].position = waiting_pos;
        }

        system.add_vehicle_to_queue(&mut vehicles[2]);
        system.update_queued_vehicle_positions(&mut vehicles);

        let waiting_pos = system.waiting_positions[&Direction::North];
        let spacing = system.queue_spacing_min;

        assert!((vehicles[1].position.y - (waiting_pos.y - spacing)).abs() < 1.0);
        assert!((vehicles[2].position.y - (waiting_pos.y - 2.0 * spacing)).abs() < 1.5);
    }

    #[test]
    fn vehicle_not_requeued_after_served() {
        let mut system = IntersectionEntrySystem::new(Vector2::new(400.0, 300.0), 200.0);
        let mut conflict_system = ConflictSystem::new();

        let mut vehicle = make_test_vehicle(100, Direction::North);
        vehicle.state = VehicleState::Moving;
        vehicle.speed = 0.0;
        vehicle.target_speed = vehicle.kinematics.v_levels[1];

        let mut vehicles = vec![vehicle];

        // Initial update should queue and immediately serve the vehicle
        system.update(&mut vehicles, 0.0, &mut conflict_system);

        assert_eq!(vehicles[0].state, VehicleState::Moving);
        assert!(system
            .entry_queues
            .get(&Direction::North)
            .and_then(|queue| queue.current_serving)
            .is_some());

        // Simulate a few ticks where the vehicle is still close to the waiting line
        for step in 1..=4 {
            // Mimic physics update driving the vehicle forward a bit each frame
            vehicles[0].target_speed = vehicles[0].kinematics.v_levels[2];
            vehicles[0].update_speed(0.2);
            vehicles[0].update_from_path(0.2);

            system.update(&mut vehicles, step as f32 * 0.2, &mut conflict_system);

            assert_ne!(vehicles[0].state, VehicleState::Waiting);
            assert!(system
                .entry_queues
                .get(&Direction::North)
                .map(|queue| queue
                    .vehicles
                    .iter()
                    .all(|qv| qv.vehicle_id != vehicles[0].id))
                .unwrap_or(true));
        }
    }
}
