use crate::statistics::SimulationStats;

pub struct StatisticsCalculator;

impl StatisticsCalculator {
    
    pub fn calculate_average_speed(stats: &SimulationStats) -> f32 {
        if stats.vehicles_passed == 0 {
            0.0
        } else {
            (stats.max_speed + stats.min_speed) / 2.0
        }
    }

    pub fn calculate_average_time(stats: &SimulationStats) -> f32 {
        if stats.vehicles_passed == 0 {
            0.0
        } else {
            (stats.max_time + stats.min_time) / 2.0
        }
    }

    pub fn calculate_close_call_rate(stats: &SimulationStats) -> f32 {
        if stats.vehicles_passed == 0 {
            0.0
        } else {
            stats.close_calls as f32 / stats.vehicles_passed as f32
        }
    }
}