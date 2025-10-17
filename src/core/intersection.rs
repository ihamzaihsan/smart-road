use crate::components::Vehicle;
use crate::core::geometry::segment_distance;
use crate::core::waypoints::Track;
use crate::resources::center_separation;

/// Spatial route reservations: compatible paths may cross concurrently.
/// A reservation has no timeout; only physical clearance releases it.
pub struct IntersectionManager {
    conflicts: Vec<Vec<bool>>,
    active: Vec<(u64, usize)>,
}

impl IntersectionManager {
    pub fn new(tracks: &[Track]) -> Self {
        let conflicts = tracks
            .iter()
            .map(|a| {
                tracks
                    .iter()
                    .map(|b| {
                        a.reserved_path.windows(2).any(|sa| {
                            b.reserved_path.windows(2).any(|sb| {
                                segment_distance(sa[0], sa[1], sb[0], sb[1]) < center_separation()
                            })
                        })
                    })
                    .collect()
            })
            .collect();
        Self {
            conflicts,
            active: Vec::new(),
        }
    }

    pub fn update(&mut self, vehicles: &mut [Vehicle], tracks: &[Track]) {
        self.active.retain(|&(id, track)| {
            vehicles
                .iter()
                .any(|v| v.id == id && v.progress < tracks[track].clear_progress)
        });
        let mut candidates: Vec<usize> = vehicles
            .iter()
            .enumerate()
            .filter(|(_, v)| {
                !v.admitted && tracks[v.track_index].stop_progress - v.progress <= 60.0
            })
            .map(|(index, _)| index)
            .collect();
        // FIFO by detection time (IDs are monotonic). An older blocked candidate
        // also blocks conflicting newcomers, preventing starvation under load.
        candidates.sort_by_key(|&i| vehicles[i].id);
        let mut deferred: Vec<usize> = Vec::new();
        for index in candidates {
            let vehicle = &mut vehicles[index];
            let track = vehicle.track_index;
            if self
                .active
                .iter()
                .any(|&(_, active)| self.conflicts[track][active])
                || deferred.iter().any(|&other| self.conflicts[track][other])
            {
                deferred.push(track);
            } else {
                vehicle.admitted = true;
                self.active.push((vehicle.id, track));
            }
        }
    }

    pub fn active_count(&self) -> usize {
        self.active.len()
    }
}
