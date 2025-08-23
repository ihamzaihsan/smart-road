use bevy::prelude::*;
use crate::resources::RoadConfig;

/// Startup system: create road geometry and spawn visual entities.
/// Uses the `RoadConfig` resource for lane width, dash length/gap, and lanes per direction.
pub fn setup_roads(mut commands: Commands, config: Res<RoadConfig>) {
    // Configuration for the intersection
    let world_size = 800.0f32; // overall viewport/world size (still constant here)
    let lane_width = config.lane_width; // width per lane (from RoadConfig)
    let lanes_per_direction = config.num_lanes; // lanes per direction (3 lanes per direction)
    let total_lanes = lanes_per_direction * 2; // both directions (6 lanes total)
    let road_width = lane_width * total_lanes as f32; // total road width (300 units)
    
    // Lane layout (with 3 lanes per direction):
    // Left side: 3 lanes moving one direction (e.g., South/West)
    // Yellow median separator in the middle
    // Right side: 3 lanes moving opposite direction (e.g., North/East)
    // Dashed lines separate individual lanes within each direction

    // Background (neutral grey)
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::rgb(0.65, 0.65, 0.65),
            custom_size: Some(Vec2::new(world_size, world_size)),
            ..default()
        },
        ..default()
    });

    // Vertical road (centered)
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::rgb(0.15, 0.15, 0.15),
            custom_size: Some(Vec2::new(road_width, world_size)),
            ..default()
        },
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
        ..default()
    });

    // Horizontal road (centered)
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::rgb(0.15, 0.15, 0.15),
            custom_size: Some(Vec2::new(world_size, road_width)),
            ..default()
        },
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
        ..default()
    });

    // Lane markings: draw separators and dashed lines between lanes
    let dash_len = config.dash_length;
    let gap = config.dash_gap;
    let intersection_half = road_width / 2.0; // half-size of the central square where roads cross

    // Compute left edge and lane centers (from left to right)
    let left_edge = -road_width / 2.0;
    let mut lane_centers: Vec<f32> = Vec::new();
    for idx in 0..total_lanes {
        lane_centers.push(left_edge + lane_width / 2.0 + idx as f32 * lane_width);
    }

    // Draw solid median separators between opposing directions
    let separator_width = 8.0f32;
    
    // Vertical median (separates left/right moving lanes at x=0)
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::rgb(0.9, 0.9, 0.1), // Yellow median
            custom_size: Some(Vec2::new(separator_width, world_size)),
            ..default()
        },
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 2.0)),
        ..default()
    });

    // Horizontal median (separates up/down moving lanes at y=0)
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::rgb(0.9, 0.9, 0.1), // Yellow median
            custom_size: Some(Vec2::new(world_size, separator_width)),
            ..default()
        },
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 2.0)),
        ..default()
    });

    // Draw dashed lines between individual lanes within each direction
    // First, compute all lane separator positions (between individual lanes, not between directions)
    let mut lane_separator_offsets: Vec<f32> = Vec::new();
    
    // Separators within the left group (lanes 0-2, separators at positions 1 and 2)
    for j in 1..lanes_per_direction {
        lane_separator_offsets.push(left_edge + j as f32 * lane_width);
    }
    
    // Separators within the right group (lanes 3-5, separators at positions 4 and 5)
    for j in (lanes_per_direction + 1)..total_lanes {
        lane_separator_offsets.push(left_edge + j as f32 * lane_width);
    }

    // Draw dashed separators for each lane separator offset, but skip inside intersection
    for &offset in &lane_separator_offsets {
        // vertical dashes (for the horizontal traffic lanes)
        let mut y = -world_size / 2.0;
        while y < world_size / 2.0 {
            let center_y = y + dash_len / 2.0;
            // Skip dashes that would be in the intersection area
            if center_y.abs() > intersection_half {
                commands.spawn(SpriteBundle {
                    sprite: Sprite {
                        color: Color::WHITE,
                        custom_size: Some(Vec2::new(3.0, dash_len)),
                        ..default()
                    },
                    transform: Transform::from_translation(Vec3::new(offset, center_y, 1.0)),
                    ..default()
                });
            }
            y += dash_len + gap;
        }

        // horizontal dashes (for the vertical traffic lanes)
        let mut x = -world_size / 2.0;
        while x < world_size / 2.0 {
            let center_x = x + dash_len / 2.0;
            // Skip dashes that would be in the intersection area
            if center_x.abs() > intersection_half {
                commands.spawn(SpriteBundle {
                    sprite: Sprite {
                        color: Color::WHITE,
                        custom_size: Some(Vec2::new(dash_len, 3.0)),
                        ..default()
                    },
                    transform: Transform::from_translation(Vec3::new(center_x, offset, 1.0)),
                    ..default()
                });
            }
            x += dash_len + gap;
        }
    }

    // Draw per-approach solid borders only for lanes that move towards the square
    let border_thickness = 6.0f32;
    // Determine groups by lane centers: first lanes_per_direction are the left approach (from left edge),
    // the last lanes_per_direction are the right approach.
    let left_centers: Vec<f32> = lane_centers.iter().cloned().take(lanes_per_direction).collect();
    let right_centers: Vec<f32> = lane_centers.iter().cloned().skip(lanes_per_direction).collect();

    // approaching from north (top): use left_centers group (move border to left lanes)
    let left_center_x = left_centers.iter().sum::<f32>() / left_centers.len() as f32;
    let approaching_width = lane_width * lanes_per_direction as f32;
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::WHITE,
            custom_size: Some(Vec2::new(approaching_width, border_thickness)),
            ..default()
        },
        transform: Transform::from_translation(Vec3::new(left_center_x, intersection_half + border_thickness / 2.0, 3.0)),
        ..default()
    });

    // approaching from south (bottom): use right_offsets group (move border to right lanes)
    let right_center_x = right_centers.iter().sum::<f32>() / right_centers.len() as f32;
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::WHITE,
            custom_size: Some(Vec2::new(approaching_width, border_thickness)),
            ..default()
        },
        transform: Transform::from_translation(Vec3::new(right_center_x, -intersection_half - border_thickness / 2.0, 3.0)),
        ..default()
    });

    // approaching from west (left): lanes moving east -> use left_centers group (y center)
    let left_center_y = left_center_x; // reuse; left_centers average is symmetric in this simplified layout
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::WHITE,
            custom_size: Some(Vec2::new(border_thickness, approaching_width)),
            ..default()
        },
        transform: Transform::from_translation(Vec3::new(-intersection_half - border_thickness / 2.0, left_center_y, 3.0)),
        ..default()
    });

    // approaching from east (right): lanes moving west -> use right_offsets group (y center)
    let right_center_y = right_center_x;
    commands.spawn(SpriteBundle {
        sprite: Sprite {
            color: Color::WHITE,
            custom_size: Some(Vec2::new(border_thickness, approaching_width)),
            ..default()
        },
        transform: Transform::from_translation(Vec3::new(intersection_half + border_thickness / 2.0, right_center_y, 3.0)),
        ..default()
    });
}
