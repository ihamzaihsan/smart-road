use crate::statistics::SimulationStats;

pub struct StatisticsReporter;

impl StatisticsReporter {
    
    pub fn generate_report(stats: &SimulationStats) -> String {
        format!(
            "=== SMART ROAD SIMULATION STATISTICS ===\n\
            \n\
            Vehicle Statistics:\n\
            - Total vehicles passed: {}\n\
            - Maximum speed: {:.1} pixels/sec\n\
            - Minimum speed: {:.1} pixels/sec\n\
            \n\
            Timing Statistics:\n\
            - Maximum passage time: {:.2} seconds\n\
            - Minimum passage time: {:.2} seconds\n\
            - Total simulation time: {:.2} seconds\n\
            \n\
            Safety Statistics:\n\
            - Close calls: {}\n\
            - Close call rate: {:.2}%\n\
            \n\
            ========================================",
            stats.vehicles_passed,
            stats.max_speed,
            stats.min_speed,
            stats.max_time,
            stats.min_time,
            stats.get_total_time(),
            stats.close_calls,
            (stats.close_calls as f32 / stats.vehicles_passed.max(1) as f32) * 100.0
        )
    }

    pub fn print_stats(stats: &SimulationStats) {
        println!("{}", Self::generate_report(stats));
    }
}