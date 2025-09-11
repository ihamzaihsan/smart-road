use crate::{
    components::Vehicle,
    core::physics::VehiclePhysics,
    resources::{stats::Stats, Config},
};
use legion::{world::SubWorld, *};
use std::collections::HashSet;

#[system]
#[read_component(Vehicle)]
pub fn collision_detection(world: &mut SubWorld, #[resource] stats: &mut Stats) {
    let mut vehicle_ids_to_remove = HashSet::new();
    let mut query = <&Vehicle>::query();
    let vehicles: Vec<&Vehicle> = query.iter(world).collect();

    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let v1 = vehicles[i];
            let v2 = vehicles[j];

            if VehiclePhysics::are_on_collision_course(v1, v2) {
                let distance = (v1.position - v2.position).magnitude();
                if distance < 10.0 {
                    stats.collisions += 1;
                    vehicle_ids_to_remove.insert(v1.id);
                    vehicle_ids_to_remove.insert(v2.id);
                }
            }
        }
    }

    if !vehicle_ids_to_remove.is_empty() {
        let mut query = <(Entity, &Vehicle)>::query();
        let entities_to_remove: Vec<Entity> = query
            .iter(world)
            .filter(|(_, v)| vehicle_ids_to_remove.contains(&v.id))
            .map(|(e, _)| *e)
            .collect();

        for entity in entities_to_remove {
            // Instead of removing the entity, which is not possible in a SubWorld,
            // you might want to mark it for removal, e.g., by adding a component.
            // For now, we just count the collision.
        }
    }
}
