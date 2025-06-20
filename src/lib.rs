use std::env;

use eframe::egui::{self, Color32, Pos2};
use egui_plot::{Line, Plot, PlotPoint, Points};
use rapier2d::{na::Unit, prelude::*};

use crate::app::{AppInfo, Camera, EngineInfo, FpsCounter, PhysicsApp};

mod app;

pub fn get_app_by_env_args() -> PhysicsApp {
    match app_id_from_env_args() {
        0 => app_ball_and_cuboid(),
        1 => app_bat_hitting_ball(),
        _ => panic!("Unknown app id"),
    }
}

fn app_id_from_env_args() -> u32 {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        println!("ID is not provided. Using default value 0.");
        0
    } else {
        args[1].parse().unwrap_or_else(|_| {
            println!("Cannot parse ID. Using default value 0.");
            0
        })
    }
}

fn app_ball_and_cuboid() -> PhysicsApp {
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
    let render_extra_ui: Box<dyn for<'a, 'b> FnMut(&'a mut EngineInfo, &'b egui::Context)> =
        Box::new(move |engine_info, ctx| {
            egui::Window::new("v-t plot").show(ctx, |ui| {
                let body = engine_info
                    .rigid_body_set
                    .iter()
                    .map(|(_, body)| body)
                    .collect::<Vec<_>>()[1];

                Plot::new("v-t plot")
                    .x_axis_label("t")
                    .y_axis_label("v")
                    .show(ui, |plot_ui| {
                        plot_ui.line(Line::new("line", points.as_slice()));
                    });

                if let Some(last) = points.last() {
                    if last.x as f32 == engine_info.sim_time {
                        // don't push duplicates
                        return;
                    }
                }

                points.push(PlotPoint::new(
                    engine_info.sim_time,
                    body.linvel().magnitude(),
                ));
            });
        });

    PhysicsApp {
        engine_info,
        app_info: AppInfo::default(),
        extra_updates: Some(render_extra_ui),
    }
}

fn app_bat_hitting_ball() -> PhysicsApp {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut impulse_joint_set = ImpulseJointSet::new();

    // Ball
    let mut l = 0.2;
    let ball_mass = 1.0;
    let ball_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, l])
        .build();
    let ball_collider = ColliderBuilder::ball(0.02)
        .restitution(1.0)
        .friction(0.0)
        .mass(ball_mass)
        .build();
    let ball_body_handle = rigid_body_set.insert(ball_body);
    collider_set.insert_with_parent(ball_collider, ball_body_handle, &mut rigid_body_set);

    // Bat
    let bat_mass = 1.5;
    let bat_length = 1.0;
    let bat_angular_inertia = 1.0 / 3.0 * bat_mass * bat_length * bat_length;
    let bat_body = RigidBodyBuilder::dynamic()
        .translation(vector![bat_length / 2.0, 0.0])
        .build();
    let bat_collider = ColliderBuilder::cuboid(bat_length / 2.0, 0.02)
        .restitution(1.0)
        .friction(0.0)
        .mass_properties(MassProperties::new(
            point![bat_length / 2.0, 0.0],
            bat_mass,
            bat_angular_inertia,
        ))
        .build();
    let bat_body_handle = rigid_body_set.insert(bat_body);
    collider_set.insert_with_parent(bat_collider, bat_body_handle, &mut rigid_body_set);

    // static body
    let static_body = RigidBodyBuilder::fixed().build();
    let static_body_handle = rigid_body_set.insert(static_body);

    // revolute joint
    let target_angvel = 1.0;
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(point![-bat_length / 2.0, 0.0])
        .local_anchor2(point![0.0, 0.0])
        .motor_velocity(-target_angvel, 1000.0)
        .motor_max_force(1000.0);
    let joint_handle = impulse_joint_set.insert(bat_body_handle, static_body_handle, joint, true);

    // Plot things
    let prediction_points = (0..100)
        .map(|i| {
            let x = 0.01 * i as f32;
            let y = (2.0 * bat_angular_inertia * x * target_angvel)
                / (bat_angular_inertia + ball_mass * x * x);
            // [x as f64, y as f64]
            PlotPoint::new(x as f64, y as f64)
        })
        .collect::<Vec<_>>();

    let mut velocity_points = Vec::new();
    let mut angular_momentum_points = Vec::new();
    let mut energy_points: Vec<PlotPoint> = Vec::new();

    let extra_updates: Box<dyn for<'a, 'b> FnMut(&'a mut EngineInfo, &'b egui::Context)> =
        Box::new(move |engine_info, ctx| {
            // Physics updates
            let bat_angle = engine_info
                .rigid_body_set
                .get(bat_body_handle)
                .unwrap()
                .rotation()
                .angle();
            let bat_angvel = engine_info
                .rigid_body_set
                .get(bat_body_handle)
                .unwrap()
                .angvel();
            let ball_vel = engine_info
                .rigid_body_set
                .get(ball_body_handle)
                .unwrap()
                .linvel()
                .magnitude();
            let joint = engine_info
                .impulse_joint_set
                .get_mut(joint_handle, true)
                .unwrap();

            if bat_angvel > target_angvel - 0.0001 {
                joint.data.set_motor_max_force(JointAxis::AngX, 0.0);
            }

            if bat_angle > 1.88 {
                let ball_linvel_mag = engine_info
                    .rigid_body_set
                    .get(ball_body_handle)
                    .unwrap()
                    .linvel()
                    .magnitude();
                let point = PlotPoint::new(l, ball_linvel_mag);
                velocity_points.push(point);

                // Restore bodies
                {
                    let bat = engine_info.rigid_body_set.get_mut(bat_body_handle).unwrap();
                    bat.set_translation(vector![bat_length / 2.0, 0.0], true);
                    bat.set_rotation(Unit::from_angle(0.0), true);
                    bat.set_angvel(0.0, true);
                    bat.set_linvel(vector![0.0, 0.0], true);
                }
                {
                    let ball = engine_info
                        .rigid_body_set
                        .get_mut(ball_body_handle)
                        .unwrap();
                    l += 0.05;
                    ball.set_translation(vector![0.0, l], true);
                    ball.set_rotation(Unit::from_angle(0.0), true);
                    ball.set_angvel(0.0, true);
                    ball.set_linvel(vector![0.0, 0.0], true);
                }
                joint.data.set_motor_max_force(JointAxis::AngX, 1000.0);
            }

            // Plot updates
            // prediction line
            let prediction_line =
                Line::new("Prediction Function", prediction_points.as_slice()).color(Color32::CYAN);
            egui::Window::new("v-l plot").show(ctx, |ui| {
                Plot::new("v-l plot")
                    .x_axis_label("l")
                    .y_axis_label("v")
                    .show(ui, |plot_ui| {
                        plot_ui.points(Points::new("points", velocity_points.as_slice()));
                        plot_ui.line(prediction_line);
                    });
            });

            // kinetic energy
            egui::Window::new("Ek-t plot").show(ctx, |ui| {
                Plot::new("Ek-t plot")
                    .x_axis_label("t")
                    .y_axis_label("Ek")
                    .show(ui, |plot_ui| {
                        plot_ui.line(Line::new("energy", energy_points.as_slice()));
                    });
            });

            // angular momentum
            egui::Window::new("L-t plot").show(ctx, |ui| {
                Plot::new("L-t plot")
                    .x_axis_label("t")
                    .y_axis_label("L")
                    .show(ui, |plot_ui| {
                        plot_ui.line(Line::new(
                            "angular momentum",
                            angular_momentum_points.as_slice(),
                        ));
                    });
            });

            if let Some(last) = energy_points.last() {
                if last.x as f32 == engine_info.sim_time {
                    // don't push duplicates
                    return;
                }
            }
            let total_kinetic_energy = engine_info
                .rigid_body_set
                .get(bat_body_handle)
                .unwrap()
                .kinetic_energy()
                + engine_info
                    .rigid_body_set
                    .get(ball_body_handle)
                    .unwrap()
                    .kinetic_energy();
            energy_points.push(PlotPoint::new(engine_info.sim_time, total_kinetic_energy));

            let total_ang_momentum = l * ball_mass * ball_vel + bat_angular_inertia * bat_angvel;
            angular_momentum_points.push(PlotPoint::new(engine_info.sim_time, total_ang_momentum));
        });

    let camera = Camera {
        center: Pos2::new(0.0, 0.0),
        zoom: 100.0,
    };

    let engine_info = EngineInfo {
        rigid_body_set,
        collider_set,
        impulse_joint_set,
        camera,
        physics_pipeline: PhysicsPipeline::new(),
        gravity: vector![0.0, 0.0],
        integration_parameters: IntegrationParameters::default(),
        island_manager: IslandManager::new(),
        broad_phase: DefaultBroadPhase::new(),
        narrow_phase: NarrowPhase::new(),
        multibody_joint_set: MultibodyJointSet::new(),
        ccd_solver: CCDSolver::new(),
        query_pipeline: QueryPipeline::new(),
        physics_hooks: (),
        event_handler: (),
        fps_counter: FpsCounter::default(),
        sim_time: 0.0,
    };

    PhysicsApp {
        engine_info,
        app_info: AppInfo::default(),
        extra_updates: Some(extra_updates),
    }
}
