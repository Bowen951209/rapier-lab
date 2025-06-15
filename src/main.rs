mod app;

use eframe::egui::{self, Pos2};
use egui_plot::{Line, Plot, PlotPoint};
use rapier2d::prelude::*;

use crate::app::{AppInfo, Camera, EngineInfo, FpsCounter, PhysicsApp};

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(vector![0.0, 0.0])
        .build();
    let collider = ColliderBuilder::cuboid(5.0, 1.0).restitution(1.0).build();
    let rigid_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);

    /* Create the bouncing ball. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 2.0])
        .build();
    let collider = ColliderBuilder::ball(0.5).restitution(1.0).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

    let camera = Camera {
        center: Pos2::new(0.0, 0.0),
        zoom: 100.0,
    };

    let engine_info = EngineInfo {
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
        fps_counter: FpsCounter::default(),
        sim_time: 0.0,
    };

    let mut points = Vec::new();
    let render_extra_ui: Box<dyn for<'a, 'b> FnMut(&'a EngineInfo, &'b egui::Context)> =
        Box::new(move |engine_info: &EngineInfo, ctx| {
            egui::Window::new("Plot").show(ctx, |ui| {
                let body = engine_info
                    .rigid_body_set
                    .iter()
                    .map(|(_, body)| body)
                    .collect::<Vec<_>>()[1];

                let point = PlotPoint::new(engine_info.sim_time, body.linvel().magnitude());
                points.push(point);

                Plot::new("v-t plot").show(ui, |plot_ui| {
                    plot_ui.line(Line::new("a", points.as_slice()));
                });
            });
        });

    let app = PhysicsApp {
        engine_info,
        app_info: AppInfo::default(),
        render_extra_ui: Some(render_extra_ui),
    };

    app.run("Rapier Lab").expect("Failed to run the frame");
}
