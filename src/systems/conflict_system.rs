use crate::components::{Direction, Route};
use nalgebra::Vector2;
use std::collections::{BTreeSet, HashMap};

/// Represents a conflict cell where vehicle paths intersect
#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub struct ConflictCellId(pub u8);

/// Represents when a vehicle's path enters and exits a conflict cell
#[derive(Debug, Clone)]
pub struct ConflictSpan {
    pub cell: ConflictCellId,
    pub s_in: f32,   // Path parameter where vehicle enters this cell
    pub s_out: f32,  // Path parameter where vehicle exits this cell
}

/// Each path is parameterized by arc-length s in [0, s_total]
#[derive(Debug, Clone)]
pub struct Path {
    pub waypoints: Vec<Vector2<f32>>,
    pub s_total: f32,
    pub conflict_spans: Vec<ConflictSpan>,
}

impl Path {
    pub fn new(waypoints: Vec<Vector2<f32>>) -> Self {
        let s_total = Self::calculate_total_length(&waypoints);
        Self {
            waypoints,
            s_total,
            conflict_spans: Vec::new(),
        }
    }

    fn calculate_total_length(waypoints: &[Vector2<f32>]) -> f32 {
        let mut total = 0.0;
        for i in 1..waypoints.len() {
            total += (waypoints[i] - waypoints[i - 1]).magnitude();
        }
        total
    }

    /// Get position and heading at parameter s along the path
    pub fn pose_at_s(&self, s: f32) -> (Vector2<f32>, Vector2<f32>) {
        if self.waypoints.len() < 2 {
            return (Vector2::zeros(), Vector2::new(1.0, 0.0));
        }

        let s_clamped = s.clamp(0.0, self.s_total);
        let mut current_s = 0.0;

        for i in 1..self.waypoints.len() {
            let segment_start = self.waypoints[i - 1];
            let segment_end = self.waypoints[i];
            let segment_length = (segment_end - segment_start).magnitude();

            if current_s + segment_length >= s_clamped {
                // We're in this segment
                let t = if segment_length > 0.0 {
                    (s_clamped - current_s) / segment_length
                } else {
                    0.0
                };
                let position = segment_start.lerp(&segment_end, t);
                let heading = if segment_length > 0.0 {
                    (segment_end - segment_start).normalize()
                } else {
                    Vector2::new(1.0, 0.0)
                };
                return (position, heading);
            }
            current_s += segment_length;
        }

        // If we get here, return the last point
        let last_idx = self.waypoints.len() - 1;
        let heading = if last_idx > 0 {
            (self.waypoints[last_idx] - self.waypoints[last_idx - 1]).normalize()
        } else {
            Vector2::new(1.0, 0.0)
        };
        (self.waypoints[last_idx], heading)
    }

    /// Get the path parameter s for the current vehicle position
    pub fn get_s_for_position(&self, position: Vector2<f32>) -> f32 {
        let mut best_s = 0.0;
        let mut min_distance = f32::INFINITY;
        let mut current_s = 0.0;

        for i in 1..self.waypoints.len() {
            let segment_start = self.waypoints[i - 1];
            let segment_end = self.waypoints[i];
            let segment_length = (segment_end - segment_start).magnitude();

            // Find closest point on this segment
            let to_point = position - segment_start;
            let segment_dir = if segment_length > 0.0 {
                (segment_end - segment_start) / segment_length
            } else {
                Vector2::zeros()
            };

            let t = if segment_length > 0.0 {
                (to_point.dot(&segment_dir) / segment_length).clamp(0.0, 1.0)
            } else {
                0.0
            };

            let closest_point = segment_start.lerp(&segment_end, t);
            let distance = (position - closest_point).magnitude();

            if distance < min_distance {
                min_distance = distance;
                best_s = current_s + t * segment_length;
            }

            current_s += segment_length;
        }

        best_s
    }
}

/// Kinematic parameters for ETA calculations
#[derive(Debug, Clone)]
pub struct Kinematics {
    pub v: f32,           // current speed (m/s)
    pub a_max: f32,       // acceleration m/s^2
    pub d_max: f32,       // braking m/s^2
    pub v_levels: [f32; 3], // [slow, med, fast] speeds
}

impl Default for Kinematics {
    fn default() -> Self {
        Self {
            v: 0.0,
            a_max: 400.0,
            d_max: 2000.0,
            v_levels: [60.0, 150.0, 300.0], // slow, med, fast
        }
    }
}

impl Kinematics {
    /// Minimal ETA assuming current v and bounded accel
    pub fn eta_to_s(&self, s_now: f32, s_target: f32) -> f32 {
        let ds = (s_target - s_now).max(0.0);
        if self.v > 0.1 {
            ds / self.v
        } else {
            ds / self.v_levels[0] + 0.2
        }
    }
}

/// A reservation for a conflict cell
#[derive(Debug, Clone)]
pub struct Reservation {
    pub t_in: f32,
    pub t_out: f32,
    pub vehicle_id: u64,
}

impl PartialEq for Reservation {
    fn eq(&self, other: &Self) -> bool {
        self.vehicle_id == other.vehicle_id && 
        (self.t_in - other.t_in).abs() < 1e-6 &&
        (self.t_out - other.t_out).abs() < 1e-6
    }
}

impl Eq for Reservation {}

impl PartialOrd for Reservation {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Reservation {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Primary: by start time
        let t_ord = self.t_in.partial_cmp(&other.t_in).unwrap_or(std::cmp::Ordering::Equal);
        if t_ord != std::cmp::Ordering::Equal {
            return t_ord;
        }
        // Secondary: by vehicle ID for consistency
        self.vehicle_id.cmp(&other.vehicle_id)
    }
}

/// Calendar for managing reservations per conflict cell
pub type CellCalendar = BTreeSet<(u32, Reservation)>; // keyed by t_in * 1000 for ordering

/// Conflict system managing intersection crossings
pub struct ConflictSystem {
    /// Map from (approach, route) to conflict spans
    pub conflict_map: HashMap<(Direction, Route), Vec<ConflictSpan>>,
    /// Calendar per conflict cell
    pub cell_calendars: HashMap<ConflictCellId, CellCalendar>,
    /// Paths for each route combination
    pub paths: HashMap<(Direction, Route), Path>,
}

impl ConflictSystem {
    pub fn new() -> Self {
        let mut system = Self {
            conflict_map: HashMap::new(),
            cell_calendars: HashMap::new(),
            paths: HashMap::new(),
        };
        system.initialize_conflict_cells();
        system
    }

    /// Initialize conflict cells for a 4-way intersection
    fn initialize_conflict_cells(&mut self) {
        // Define conflict cell IDs
        const STRAIGHT_CORE: ConflictCellId = ConflictCellId(0);
        const NORTH_LEFT_ARC: ConflictCellId = ConflictCellId(1);
        const SOUTH_LEFT_ARC: ConflictCellId = ConflictCellId(2);
        const EAST_LEFT_ARC: ConflictCellId = ConflictCellId(3);
        const WEST_LEFT_ARC: ConflictCellId = ConflictCellId(4);
        const NORTH_RIGHT_ARC: ConflictCellId = ConflictCellId(5);
        const SOUTH_RIGHT_ARC: ConflictCellId = ConflictCellId(6);
        const EAST_RIGHT_ARC: ConflictCellId = ConflictCellId(7);
        const WEST_RIGHT_ARC: ConflictCellId = ConflictCellId(8);

        // Initialize calendars for each cell
        for id in 0..=8 {
            self.cell_calendars.insert(ConflictCellId(id), BTreeSet::new());
        }

        // Define conflict spans for each route
        // These would be calculated based on actual intersection geometry
        // For now, using representative values

        // North-South straight routes cross the core
        self.conflict_map.insert(
            (Direction::North, Route::Straight),
            vec![ConflictSpan {
                cell: STRAIGHT_CORE,
                s_in: 150.0,  // Enter intersection
                s_out: 250.0, // Exit intersection
            }],
        );

        self.conflict_map.insert(
            (Direction::South, Route::Straight),
            vec![ConflictSpan {
                cell: STRAIGHT_CORE,
                s_in: 150.0,
                s_out: 250.0,
            }],
        );

        // East-West straight routes cross the core
        self.conflict_map.insert(
            (Direction::East, Route::Straight),
            vec![ConflictSpan {
                cell: STRAIGHT_CORE,
                s_in: 150.0,
                s_out: 250.0,
            }],
        );

        self.conflict_map.insert(
            (Direction::West, Route::Straight),
            vec![ConflictSpan {
                cell: STRAIGHT_CORE,
                s_in: 150.0,
                s_out: 250.0,
            }],
        );

        // Left turns have their own arc cells but may also cross core
        self.conflict_map.insert(
            (Direction::North, Route::Left),
            vec![
                ConflictSpan {
                    cell: NORTH_LEFT_ARC,
                    s_in: 140.0,
                    s_out: 190.0,
                },
                ConflictSpan {
                    cell: STRAIGHT_CORE,
                    s_in: 160.0,
                    s_out: 180.0,
                },
            ],
        );

        // Add similar definitions for other left turns...
        // Right turns typically don't conflict as much
        self.conflict_map.insert(
            (Direction::North, Route::Right),
            vec![ConflictSpan {
                cell: NORTH_RIGHT_ARC,
                s_in: 140.0,
                s_out: 190.0,
            }],
        );

        // Add similar definitions for other directions and routes...
    }

    /// Try to reserve a conflict cell for a vehicle
    pub fn reserve_cell(
        &mut self,
        cell_id: ConflictCellId,
        vehicle_id: u64,
        ask_in: f32,
        duration: f32,
        separation: f32,
    ) -> Result<(f32, f32), String> {
        let calendar = self.cell_calendars.get_mut(&cell_id).unwrap();
        
        let mut t_in = ask_in;
        let max_iterations = 20; // Prevent infinite loops
        let mut iterations = 0;

        loop {
            if iterations >= max_iterations {
                return Err("Could not find available slot".to_string());
            }
            iterations += 1;

            let t_out = t_in + duration;
            let t_in_key = (t_in * 1000.0) as u32;

            // Check for conflicts with existing reservations
            let mut conflict = false;

            // Check reservation that might end after our start
            for (_, res) in calendar.range(..) {
                if res.vehicle_id == vehicle_id {
                    continue; // Skip our own reservations
                }

                let our_start = t_in;
                let our_end = t_out;
                let their_start = res.t_in;
                let their_end = res.t_out;

                // Check if windows overlap with separation
                let overlap = !(our_end + separation <= their_start || their_end + separation <= our_start);
                
                if overlap {
                    // Push our start time past their end
                    t_in = their_end + separation + 0.05;
                    conflict = true;
                    break;
                }
            }

            if !conflict {
                // No conflict found, make the reservation
                let reservation = Reservation {
                    t_in,
                    t_out,
                    vehicle_id,
                };
                calendar.insert((t_in_key, reservation));
                return Ok((t_in, t_out));
            }
        }
    }

    /// Clean up expired reservations
    pub fn cleanup_expired_reservations(&mut self, current_time: f32) {
        for calendar in self.cell_calendars.values_mut() {
            calendar.retain(|(_, res)| res.t_out > current_time);
        }
    }

    /// Get conflict spans for a vehicle's route
    pub fn get_conflict_spans(&self, approach: Direction, route: Route) -> Option<&Vec<ConflictSpan>> {
        self.conflict_map.get(&(approach, route))
    }

    /// Calculate priority for conflict resolution
    pub fn calculate_priority(_approach: Direction, route: Route, vehicle_id: u64) -> u32 {
        let route_priority = match route {
            Route::Right => 3,    // Highest priority
            Route::Straight => 2,
            Route::Left => 1,     // Lowest priority
        };
        
        // Combine route priority with vehicle ID for tie-breaking
        (route_priority << 16) | ((vehicle_id & 0xFFFF) as u32)
    }
}

impl Default for ConflictSystem {
    fn default() -> Self {
        Self::new()
    }
}
