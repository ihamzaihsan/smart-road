use crate::components::{Direction, Route};
use crate::resources::{RoadConfig, SimulationConfig};
use crate::systems::SpawnManager;
use nalgebra::Vector2;

/// Generate waypoints for a vehicle's path through the intersection
pub fn generate_waypoints(
    direction: Direction,
    lane_index: usize,
    route: Route,
    start_pos: Vector2<f32>,
    road_config: &RoadConfig,
    simulation_config: &SimulationConfig,
    spawn_manager: &SpawnManager,
) -> Vec<Vector2<f32>> {
    let mut waypoints = Vec::new();
    waypoints.push(start_pos);

    let max_lane_index = road_config.num_lanes.saturating_sub(1);
    let clamped_lane = lane_index.min(max_lane_index);
    let intersection_half = road_config.lane_width * road_config.num_lanes as f32;

    let movement_axis_is_y = matches!(direction, Direction::North | Direction::South);

    match route {
        Route::Straight => {
            let entry_boundary = boundary_before(direction, intersection_half);
            let mut point = set_axis(start_pos, movement_axis_is_y, entry_boundary);
            push_if_new(&mut waypoints, point);

            let exit_boundary = boundary_after(direction, intersection_half);
            point = set_axis(point, movement_axis_is_y, exit_boundary);
            push_if_new(&mut waypoints, point);

            let exit_far = exit_far_point(
                direction,
                clamped_lane,
                simulation_config,
                road_config,
                spawn_manager,
            );
            push_if_new(&mut waypoints, exit_far);
        }
        Route::Right | Route::Left => {
            let exit_direction = match route {
                Route::Right => direction.turn_right(),
                Route::Left => direction.turn_left(),
                Route::Straight => direction,
            };

            let entry_lane_centers = spawn_manager.get_lane_centers(direction, road_config);
            let exit_lane_centers = spawn_manager.get_lane_centers(exit_direction, road_config);

            let mut entry_indices: Vec<usize> = (0..entry_lane_centers.len()).collect();
            entry_indices.sort_by(|&a, &b| {
                entry_lane_centers[a]
                    .abs()
                    .partial_cmp(&entry_lane_centers[b].abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            let mut exit_indices: Vec<usize> = (0..exit_lane_centers.len()).collect();
            exit_indices.sort_by(|&a, &b| {
                exit_lane_centers[a]
                    .abs()
                    .partial_cmp(&exit_lane_centers[b].abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            let entry_rank = entry_indices
                .iter()
                .position(|&idx| idx == clamped_lane)
                .unwrap_or(0);

            let chosen_rank = entry_rank.min(exit_indices.len().saturating_sub(1));
            let best_exit_lane = exit_indices[chosen_rank];

            let turn_coordinate =
                spawn_manager.get_lane_center_by_index(exit_direction, best_exit_lane, road_config);
            let turn_point = set_axis(start_pos, movement_axis_is_y, turn_coordinate);
            push_if_new(&mut waypoints, turn_point);

            let exit_movement_axis_is_y =
                matches!(exit_direction, Direction::North | Direction::South);
            let exit_boundary = boundary_after(exit_direction, intersection_half);
            let corner_point = set_axis(turn_point, exit_movement_axis_is_y, exit_boundary);
            push_if_new(&mut waypoints, corner_point);

            let exit_far = exit_far_point(
                exit_direction,
                best_exit_lane,
                simulation_config,
                road_config,
                spawn_manager,
            );
            push_if_new(&mut waypoints, exit_far);
        }
    }

    waypoints
}

/// Push a point to the waypoints list only if it's sufficiently different from the last point
pub fn push_if_new(points: &mut Vec<Vector2<f32>>, point: Vector2<f32>) {
    if points
        .last()
        .map_or(true, |last| (*last - point).magnitude_squared() > 1.0)
    {
        points.push(point);
    }
}

/// Set either X or Y axis of a point based on direction
pub fn set_axis(point: Vector2<f32>, use_y: bool, value: f32) -> Vector2<f32> {
    if use_y {
        Vector2::new(point.x, value)
    } else {
        Vector2::new(value, point.y)
    }
}

/// Get the boundary position before entering intersection
pub fn boundary_before(direction: Direction, half_size: f32) -> f32 {
    match direction {
        Direction::North | Direction::East => -half_size,
        Direction::South | Direction::West => half_size,
    }
}

/// Get the boundary position after exiting intersection
pub fn boundary_after(direction: Direction, half_size: f32) -> f32 {
    match direction {
        Direction::North | Direction::East => half_size,
        Direction::South | Direction::West => -half_size,
    }
}

/// Calculate the far exit point for a vehicle based on its direction and lane
pub fn exit_far_point(
    direction: Direction,
    lane_index: usize,
    simulation_config: &SimulationConfig,
    road_config: &RoadConfig,
    spawn_manager: &SpawnManager,
) -> Vector2<f32> {
    let max_lane_index = road_config.num_lanes.saturating_sub(1);
    let clamped_lane = lane_index.min(max_lane_index);
    let lane_center = spawn_manager.get_lane_center_by_index(direction, clamped_lane, road_config);
    let distance = simulation_config.cleanup_distance;
    match direction {
        Direction::North => Vector2::new(lane_center, distance),
        Direction::South => Vector2::new(lane_center, -distance),
        Direction::East => Vector2::new(distance, lane_center),
        Direction::West => Vector2::new(-distance, lane_center),
    }
}

/// Check if a path crosses the intersection core
pub fn has_intersection_point(
    path: &[Vector2<f32>],
    intersection_center: Vector2<f32>,
    radius: f32,
) -> bool {
    path.iter()
        .any(|point| (point - intersection_center).magnitude() < radius)
}
