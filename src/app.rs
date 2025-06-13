use eframe::egui::{self, Color32, Pos2, Rect, Shape, Stroke};
use rapier2d::{na::Vector2, prelude::*};

#[derive(Default)]
pub struct AppInfo {
    pub screen_rect: Option<Rect>,
}

pub struct Camera {
    pub center: Pos2,
    pub zoom: f32,
}

pub struct PhysicsApp {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub physics_pipeline: PhysicsPipeline,
    pub gravity: Vector2<Real>,
    pub integration_parameters: IntegrationParameters,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub physics_hooks: (),
    pub event_handler: (),
    pub camera: Camera,
    pub app_info: AppInfo,
}

impl PhysicsApp {
    pub fn run(self, app_name: &str) -> eframe::Result {
        let native_options = eframe::NativeOptions::default();
        eframe::run_native(app_name, native_options, Box::new(|_cc| Ok(Box::new(self))))
    }

    pub fn transform_to_screen_pos(&self, p: (f32, f32)) -> Pos2 {
        let (mut x, mut y) = (p.0, p.1);
        let screen_rect = self.app_info.screen_rect.unwrap();

        x += self.camera.center.x;
        x *= self.camera.zoom;
        x += screen_rect.width() * 0.5;

        y += self.camera.center.y;
        y *= -self.camera.zoom;
        y += screen_rect.height() * 0.5;

        Pos2::new(x, y)
    }
}

impl eframe::App for PhysicsApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.app_info.screen_rect = Some(ctx.screen_rect());

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Hello World!");

            self.physics_pipeline.step(
                &self.gravity,
                &self.integration_parameters,
                &mut self.island_manager,
                &mut self.broad_phase,
                &mut self.narrow_phase,
                &mut self.rigid_body_set,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                &mut self.ccd_solver,
                Some(&mut self.query_pipeline),
                &self.physics_hooks,
                &self.event_handler,
            );

            let painter = ui.painter();
            for (_body_handle, body) in self.rigid_body_set.iter() {
                if body.colliders().len() > 1 {
                    panic!("Multiple colliders per body not supported");
                }

                let collider = self.collider_set.get(body.colliders()[0]).unwrap();
                let shape = collider.shape().as_typed_shape();
                let position = collider.position();
                let center =
                    self.transform_to_screen_pos((position.translation.x, position.translation.y));

                match shape {
                    TypedShape::Ball(ball) => {
                        let radius = ball.radius * self.camera.zoom;
                        painter.circle(center, radius, Color32::BLUE, Stroke::NONE);

                        let line_end = position * point![ball.radius, 0.0];
                        let line_end = self.transform_to_screen_pos((line_end.x, line_end.y));
                        painter.line_segment([center, line_end], Stroke::new(1.0, Color32::WHITE));
                    }
                    TypedShape::Cuboid(cuboid) => {
                        let vertices = [
                            position * point!(-cuboid.half_extents.x, -cuboid.half_extents.y),
                            position * point![cuboid.half_extents.x, -cuboid.half_extents.y],
                            position * point![cuboid.half_extents.x, cuboid.half_extents.y],
                            position * point!(-cuboid.half_extents.x, cuboid.half_extents.y),
                        ]
                        .iter()
                        .map(|v| self.transform_to_screen_pos((v.x, v.y)))
                        .collect::<Vec<_>>();

                        let up_right = vertices[2];

                        painter.add(Shape::convex_polygon(
                            vertices,
                            Color32::ORANGE,
                            Stroke::NONE,
                        ));

                        painter.line_segment([center, up_right], Stroke::new(1.0, Color32::WHITE));
                    }
                    _ => {
                        panic!("Unsupported shape");
                    }
                }
            }
        });

        ctx.request_repaint();
    }
}
