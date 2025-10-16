use nalgebra::Vector2;

/// Forward-looking detection zone for collision avoidance
#[derive(Debug, Clone, Copy)]
pub struct SensorZone {
    pub position: Vector2<f32>,
    pub width: f32,
    pub height: f32,
}

impl SensorZone {
    pub fn new(vehicle: &Vehicle, length: f32, width: f32) -> Self {
        match vehicle.direction {
            Direction::North => SensorZone {
                position: Vector2::new(
                    vehicle.position.x - width / 2.0,
                    vehicle.position.y + vehicle.length / 2.0,
                ),
                width,
                height: length,
            },
            Direction::South => SensorZone {
                position: Vector2::new(
                    vehicle.position.x - width / 2.0,
                    vehicle.position.y - vehicle.length / 2.0 - length,
                ),
                width,
                height: length,
            },
            Direction::East => SensorZone {
                position: Vector2::new(
                    vehicle.position.x + vehicle.length / 2.0,
                    vehicle.position.y - width / 2.0,
                ),
                width: length,
                height: width,
            },
            Direction::West => SensorZone {
                position: Vector2::new(
                    vehicle.position.x - vehicle.length / 2.0 - length,
                    vehicle.position.y - width / 2.0,
                ),
                width: length,
                height: width,
            },
        }
    }

    pub fn intersects_vehicle(&self, other: &Vehicle) -> bool {
        let sx = self.position.x;
        let sy = self.position.y;
        let sw = self.width;
        let sh = self.height;

        let ox = other.position.x - other.width / 2.0;
        let oy = other.position.y - other.length / 2.0;
        let ow = other.width;
        let oh = other.length;

        sx < ox + ow && sx + sw > ox && sy < oy + oh && sy + sh > oy
    }
}

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u64,
    pub position: Vector2<f32>,
    pub width: f32,
    pub length: f32,
    pub speed: f32,
    pub state: VehicleState,
    pub route: Route,
    pub lane_index: usize,
    pub direction: Direction,
    pub target_speed: f32,
    pub rotation: f32,
    pub texture_index: usize,
    pub path: Vec<Vector2<f32>>,
    pub path_index: usize,
    pub queue_target: Option<Vector2<f32>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Route {
    Straight,
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleState {
    Waiting,
    Moving,
    Stopped,
    AlwaysMoving,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    North,
    South,
    East,
    West,
}

impl Direction {
    pub fn movement_vector(&self) -> Vector2<f32> {
        match self {
            Direction::North => Vector2::new(0.0, 1.0),
            Direction::South => Vector2::new(0.0, -1.0),
            Direction::East => Vector2::new(1.0, 0.0),
            Direction::West => Vector2::new(-1.0, 0.0),
        }
    }

    pub fn rotation_angle(&self) -> f32 {
        match self {
            Direction::North => 0.0,
            Direction::South => std::f32::consts::PI,
            Direction::East => std::f32::consts::PI / 2.0,
            Direction::West => -std::f32::consts::PI / 2.0,
        }
    }

    pub fn turn_left(&self) -> Self {
        match self {
            Direction::North => Direction::West,
            Direction::South => Direction::East,
            Direction::East => Direction::North,
            Direction::West => Direction::South,
        }
    }

    pub fn turn_right(&self) -> Self {
        match self {
            Direction::North => Direction::East,
            Direction::South => Direction::West,
            Direction::East => Direction::South,
            Direction::West => Direction::North,
        }
    }

    pub fn from_vector(vector: Vector2<f32>) -> Option<Self> {
        let mag_sq = vector.x * vector.x + vector.y * vector.y;
        if mag_sq < 1e-4 {
            return None;
        }
        if vector.x.abs() > vector.y.abs() {
            if vector.x > 0.0 {
                Some(Direction::East)
            } else {
                Some(Direction::West)
            }
        } else if vector.y.abs() > vector.x.abs() {
            if vector.y > 0.0 {
                Some(Direction::North)
            } else {
                Some(Direction::South)
            }
        } else {
            None
        }
    }
}

impl Vehicle {
    pub fn get_current_waypoint(&self) -> Option<Vector2<f32>> {
        if self.path_index < self.path.len() {
            Some(self.path[self.path_index])
        } else {
            None
        }
    }

    pub fn advance_waypoint(&mut self) {
        if self.path_index < self.path.len() {
            self.path_index += 1;
        }
    }



    pub fn check_ahead(&self, all_vehicles: &[Vehicle], sensor_length: f32, sensor_width: f32) -> (bool, Option<f32>) {
        let sensor = SensorZone::new(self, sensor_length, sensor_width);
        
        let mut closest_distance: Option<f32> = None;
        let mut detected = false;

        for other in all_vehicles {
            if self.id == other.id {
                continue;
            }

            if sensor.intersects_vehicle(other) {
                let gap_distance = match self.direction {
                    Direction::North => {
                        // North moves in +Y direction
                        let my_front = self.position.y + self.length / 2.0;
                        let other_rear = other.position.y - other.length / 2.0;
                        // Gap is positive if other is ahead
                        if other_rear > my_front {
                            Some(other_rear - my_front)
                        } else {
                            Some(0.0) // Vehicles overlapping or side-by-side, emergency!
                        }
                    }
                    Direction::South => {
                        // South moves in -Y direction
                        let my_front = self.position.y - self.length / 2.0;
                        let other_rear = other.position.y + other.length / 2.0;
                        // Gap is positive if other is ahead
                        if other_rear < my_front {
                            Some(my_front - other_rear)
                        } else {
                            Some(0.0) // Vehicles overlapping or side-by-side, emergency!
                        }
                    }
                    Direction::East => {
                        // East moves in +X direction
                        let my_front = self.position.x + self.length / 2.0;
                        let other_rear = other.position.x - other.length / 2.0;
                        // Gap is positive if other is ahead
                        if other_rear > my_front {
                            Some(other_rear - my_front)
                        } else {
                            Some(0.0) // Vehicles overlapping or side-by-side, emergency!
                        }
                    }
                    Direction::West => {
                        // West moves in -X direction  
                        let my_front = self.position.x - self.length / 2.0;
                        let other_rear = other.position.x + other.length / 2.0;
                        // Gap is positive if other is ahead
                        if other_rear < my_front {
                            Some(my_front - other_rear)
                        } else {
                            Some(0.0) // Vehicles overlapping or side-by-side, emergency!
                        }
                    }
                };

                if let Some(distance) = gap_distance {
                    detected = true;
                    closest_distance = Some(match closest_distance {
                        Some(current_min) => current_min.min(distance),
                        None => distance,
                    });
                }
            }
        }

        (detected, closest_distance)
    }
    
}
