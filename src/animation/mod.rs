use std::time::Duration;

#[derive(Debug, Clone)]
pub struct Animation {
    start_time: Duration,
    duration: Duration,
    start_value: f32,
    end_value: f32,
    easing: EasingFunction,
}

#[derive(Debug, Clone, Copy)]
pub enum EasingFunction {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
}

impl Animation {
    pub fn new(
        start_time: Duration,
        duration: Duration,
        start_value: f32,
        end_value: f32,
        easing: EasingFunction,
    ) -> Self {
        Self {
            start_time,
            duration,
            start_value,
            end_value,
            easing,
        }
    }

    pub fn value_at(&self, current_time: Duration) -> f32 {
        if current_time <= self.start_time {
            return self.start_value;
        }

        let elapsed = current_time - self.start_time;
        if elapsed >= self.duration {
            return self.end_value;
        }

        let t = elapsed.as_secs_f32() / self.duration.as_secs_f32();
        let eased_t = self.apply_easing(t);
        
        self.start_value + (self.end_value - self.start_value) * eased_t
    }

    pub fn is_complete(&self, current_time: Duration) -> bool {
        current_time >= self.start_time + self.duration
    }

    fn apply_easing(&self, t: f32) -> f32 {
        match self.easing {
            EasingFunction::Linear => t,
            EasingFunction::EaseIn => t * t,
            EasingFunction::EaseOut => 1.0 - (1.0 - t) * (1.0 - t),
            EasingFunction::EaseInOut => {
                if t < 0.5 {
                    2.0 * t * t
                } else {
                    1.0 - 2.0 * (1.0 - t) * (1.0 - t)
                }
            }
        }
    }
}

pub struct AnimationManager {
    animations: Vec<(String, Animation)>,
}

impl AnimationManager {
    pub fn new() -> Self {
        Self {
            animations: Vec::new(),
        }
    }

    pub fn add_animation(&mut self, name: String, animation: Animation) {
        // Remove existing animation with the same name
        self.animations.retain(|(n, _)| n != &name);
        self.animations.push((name, animation));
    }

    pub fn get_value(&self, name: &str, current_time: Duration) -> Option<f32> {
        self.animations
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, anim)| anim.value_at(current_time))
    }

    pub fn cleanup(&mut self, current_time: Duration) {
        self.animations.retain(|(_, anim)| !anim.is_complete(current_time));
    }

    pub fn is_animating(&self, name: &str, current_time: Duration) -> bool {
        self.animations
            .iter()
            .any(|(n, anim)| n == name && !anim.is_complete(current_time))
    }
}

impl Default for AnimationManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn animation_linear() {
        let anim = Animation::new(
            Duration::from_secs(0),
            Duration::from_secs(1),
            0.0,
            10.0,
            EasingFunction::Linear,
        );

        assert_eq!(anim.value_at(Duration::from_secs(0)), 0.0);
        assert_eq!(anim.value_at(Duration::from_millis(500)), 5.0);
        assert_eq!(anim.value_at(Duration::from_secs(1)), 10.0);
        assert!(anim.is_complete(Duration::from_secs(1)));
    }

    #[test]
    fn animation_manager() {
        let mut manager = AnimationManager::new();
        let anim = Animation::new(
            Duration::from_secs(0),
            Duration::from_secs(1),
            0.0,
            10.0,
            EasingFunction::Linear,
        );

        manager.add_animation("test".to_string(), anim);
        assert_eq!(
            manager.get_value("test", Duration::from_millis(500)),
            Some(5.0)
        );
        assert!(manager.is_animating("test", Duration::from_millis(500)));
        assert!(!manager.is_animating("test", Duration::from_secs(2)));
    }
}