mod app;

use eframe::egui::Pos2;
use rapier2d::prelude::*;

use crate::app::{AppInfo, Camera, PhysicsApp};

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(vector![0.0, 0.0])
        .build();
    let collider = ColliderBuilder::cuboid(5.0, 1.0).build();
    let rigid_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);

    /* Create the bouncing ball. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 10.0])
        .build();
    let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

    let camera = Camera {
        center: Pos2::new(0.0, 0.0),
        zoom: 100.0,
    };

    let app = PhysicsApp {
        rigid_body_set,
        collider_set,
        camera,
        physics_pipeline: PhysicsPipeline::new(),
        gravity: vector![0.0, -9.81],
        integration_parameters: IntegrationParameters::default(),
        island_manager: IslandManager::new(),
        broad_phase: DefaultBroadPhase::new(),
        narrow_phase: NarrowPhase::new(),
        impulse_joint_set: ImpulseJointSet::new(),
        multibody_joint_set: MultibodyJointSet::new(),
        ccd_solver: CCDSolver::new(),
        query_pipeline: QueryPipeline::new(),
        physics_hooks: (),
        event_handler: (),
        app_info: AppInfo::default(),
    };

    app.run("Rapier Lab").expect("Failed to run the frame");
}
