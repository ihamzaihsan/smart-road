use crate::components::Vehicle;

pub struct TrafficOptimizer;

impl TrafficOptimizer {
    
    pub fn optimize_flow(vehicles: &mut [Vehicle]) {
        // TODO: Implement traffic flow optimization
        // - Minimize total wait time
        // - Maximize throughput
        // - Reduce congestion
        
        Self::balance_lane_usage(vehicles);
        Self::optimize_speeds(vehicles);
    }

    fn balance_lane_usage(_vehicles: &mut [Vehicle]) {
        // TODO: Implement lane balancing
    }

    fn optimize_speeds(vehicles: &mut [Vehicle]) {
        // Simple optimization: maintain consistent spacing
        for vehicle in vehicles.iter_mut() {
            if vehicle.speed < 20.0 {
                vehicle.target_speed = vehicle.target_speed.min(40.0);
            }
        }
    }

    pub fn calculate_efficiency(vehicles: &[Vehicle]) -> f32 {
        if vehicles.is_empty() {
            return 1.0;
        }
        
        let total_speed: f32 = vehicles.iter().map(|v| v.speed).sum();
        let average_speed = total_speed / vehicles.len() as f32;
        
        // Efficiency based on speed utilization
        (average_speed / 100.0).min(1.0)
    }
}